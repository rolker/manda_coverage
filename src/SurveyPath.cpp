/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.cpp                                  */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#include <iterator>
#include <limits>
#include <mutex>
//#include <regex>
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "manda_coverage/lib_geometry/AngleUtils.h"
#include "manda_coverage/lib_geometry/XYFormatUtilsSegl.h"
#include "manda_coverage/RecordSwath.h"
#include "manda_coverage/PathPlan.h"
#include "manda_coverage/SurveyPath.h"
#include <boost/algorithm/string.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "marine_nav_utilities/utilities.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "nav2_util/node_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"


namespace manda_coverage
{


//---------------------------------------------------------
// Constructor

SurveyPath::SurveyPath(NodeInterfaces node_interfaces)
 :node_interfaces_(node_interfaces),
  logger_(node_interfaces.get_node_logging_interface()->get_logger()),
  clock_(node_interfaces.get_node_clock_interface()->get_clock())
{

}

void SurveyPath::configure()
{
  m_swath_side = BoatSide::Stbd;
  m_swath_record.SetOutputSide(m_swath_side);

  m_recording = false;

  auto parameter_interface = node_interfaces_.get_node_parameters_interface();

  // Topic names are wired into subscriptions/publishers at configure time, so
  // they are restart-only. Marking them read_only makes ROS 2 reject any
  // `ros2 param set` on them before the on-set callback is ever invoked.
  auto declare_read_only_string = [&](const std::string & name,
                                      const std::string & default_value,
                                      const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    if(!parameter_interface->has_parameter(name))
      parameter_interface->declare_parameter(
        name, rclcpp::ParameterValue(default_value), descriptor);
  };

  declare_read_only_string("soundings_topic", "soundings",
    "Topic carrying incoming sonar soundings (read-only; restart to change)");

  std::string soundings_topic = parameter_interface->get_parameter("soundings_topic").as_string();

  auto callback_group = node_interfaces_.get_node_base_interface()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group;

  m_ping_subscription = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
    node_interfaces_,
    soundings_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SurveyPath::pingCallback, this, std::placeholders::_1),
    subscription_options
  );

  m_odom_subscription = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
    node_interfaces_,
    "odom",
    rclcpp::SensorDataQoS(),
    std::bind(&SurveyPath::odomCallback, this, std::placeholders::_1),
    subscription_options
  );

  declare_read_only_string("display_topic", "",
    "Topic for swath/path visualization markers (read-only; restart to change)");

  std::string display_topic = parameter_interface->get_parameter("display_topic").as_string();

  if(display_topic != "")
    m_display_publisher = rclcpp::create_publisher<visualization_msgs::msg::Marker>(node_interfaces_, display_topic, 10);
  else
    m_display_publisher.reset();

  // Coverage-density tuning parameters. Declared with floating-point-range
  // descriptors so the node rejects out-of-range values at declare/set time.
  auto declare_bounded = [&](const std::string & name, double default_value,
                             double from_value, double to_value,
                             const std::string & description) -> double
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = from_value;
    range.to_value = to_value;
    range.step = 0.0;
    descriptor.floating_point_range.push_back(range);
    if(!parameter_interface->has_parameter(name))
      parameter_interface->declare_parameter(
        name, rclcpp::ParameterValue(default_value), descriptor);
    return parameter_interface->get_parameter(name).as_double();
  };

  // Lower bound is a small positive epsilon, not 0.0: a zero threshold makes the
  // waypoint-reached checks in odomCallback (distance < threshold) never fire,
  // stalling transit->survey/survey->transit transitions. 0.1 m floors it below
  // any realistic survey threshold while keeping it strictly positive.
  waypoint_distance_threshold_ = declare_bounded("waypoint_distance_threshold",
    waypoint_distance_threshold_, 0.1, std::numeric_limits<double>::max(),
    "Distance threshold to consider a waypoint reached, in meters");
  lead_in_distance_ = declare_bounded("lead_in_distance",
    lead_in_distance_, 0.0, std::numeric_limits<double>::max(),
    "Lead-in distance added to the beginning of a survey line, in meters");
  lead_out_distance_ = declare_bounded("lead_out_distance",
    lead_out_distance_, 0.0, std::numeric_limits<double>::max(),
    "Lead-out distance added to the end of a survey line, in meters");

  m_swath_overlap = declare_bounded("swath_overlap", 0.2, 0.0, 1.0,
    "Fraction of swath width to overlap adjacent survey lines [0-1]");
  m_max_bend_angle = declare_bounded("max_bend_angle", 60.0, 0.0, 90.0,
    "Maximum bend angle between path segments, in degrees");
  double swath_record_interval = declare_bounded("swath_record_interval", 10.0,
    0.001, std::numeric_limits<double>::max(),
    "Distance between swath-minimum analysis intervals, in meters");
  double min_allowable_swath = declare_bounded("min_allowable_swath", 0.0,
    0.0, std::numeric_limits<double>::max(),
    "Minimum swath width treated as valid coverage, in meters");

  m_swath_record.SetInterval(swath_record_interval);
  m_swath_record.SetMinAllowableSwath(min_allowable_swath);

  // Type and range rejection are enforced by the parameter descriptors above:
  // rclcpp validates the proposed value (correct type, within the declared
  // floating-point range) and rejects it with a generic `reason` *before* any
  // callback fires. There is therefore no on-set validation callback — for
  // these statically-typed, range-bounded doubles it would be dead code.
  //
  // APPLY: runs after the set is committed, so cached members and live
  // RecordSwath state are updated only once the whole set has been accepted.
  // The lock serializes this apply against the planning readers in
  // ping/odomCallback, which run in a different callback group concurrently
  // under a MultiThreadedExecutor.
  post_set_param_callback_handle_ =
    parameter_interface->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters)
      {
        std::lock_guard<std::mutex> lock(m_param_mutex);
        for(const auto & parameter : parameters)
        {
          const auto & name = parameter.get_name();
          if(name == "waypoint_distance_threshold")
            waypoint_distance_threshold_ = parameter.as_double();
          else if(name == "lead_in_distance")
            lead_in_distance_ = parameter.as_double();
          else if(name == "lead_out_distance")
            lead_out_distance_ = parameter.as_double();
          else if(name == "swath_overlap")
            m_swath_overlap = parameter.as_double();
          else if(name == "max_bend_angle")
            m_max_bend_angle = parameter.as_double();
          else if(name == "swath_record_interval")
            m_swath_record.SetInterval(parameter.as_double());
          else if(name == "min_allowable_swath")
            m_swath_record.SetMinAllowableSwath(parameter.as_double());
        }
      });
}

void SurveyPath::activate()
{
}

void SurveyPath::deactivate()
{
}

void SurveyPath::cleanup()
{
  m_ping_subscription.reset();
  m_display_publisher.reset();
  m_odom_subscription.reset();
  post_set_param_callback_handle_.reset();
}

void SurveyPath::set_next_line_callback(std::function<void(const nav_msgs::msg::Path&, int)> next_line_callback)
{
  next_line_callback_ = next_line_callback;
}

void SurveyPath::set_done_callback(std::function<void(bool)> done_callback)
{
  done_callback_ = done_callback;
}


void SurveyPath::set_goal(const geometry_msgs::msg::PolygonStamped &goal)
{
  // Held for the whole callback: it reads lead_in_distance_/lead_out_distance_
  // (via DetermineStartAndTurn -> extendPathForLeadInOut) and mutates
  // m_swath_record (SetOutputSide, ResetLine), all of which the post-set apply
  // callback updates from another callback group. set_goal runs in the
  // action-server's default callback group, concurrent with that apply path
  // under a MultiThreadedExecutor, so it must serialize against it too. The
  // callees reached from here (DetermineStartAndTurn, extendPathForLeadInOut,
  // sendPath) must NOT re-lock — the mutex is non-recursive.
  std::lock_guard<std::mutex> lock(m_param_mutex);

  RCLCPP_INFO_STREAM(logger_, "Goal received");

  if(goal.polygon.points.empty())
  {
    m_state = idle;
    m_recording = false;
    m_op_region.clear();
    m_survey_path.clear();
    path_marker_.points.clear();
    return;
  } 

  m_map_frame = goal.header.frame_id;
  RCLCPP_INFO_STREAM(logger_, "map frame: " << m_map_frame);

  m_op_region.clear();
  m_survey_path.clear();
  path_marker_.points.clear();

  for(const auto& point: goal.polygon.points)
  {
    RCLCPP_INFO_STREAM(logger_, "  point: " << point.x << ", " << point.y);
    boost::geometry::append(m_op_region.outer(), BPoint(point.x, point.y));
    if(m_survey_path.size() < 2)
      m_survey_path.add_vertex(point.x,point.y);
  }

  boost::geometry::append(m_op_region.outer(),m_op_region.outer()[0]);

  boost::geometry::validity_failure_type failure;
  bool valid = boost::geometry::is_valid(m_op_region, failure);
  RCLCPP_INFO_STREAM(logger_, "Polygon valid: " << (valid?"yes":"no"));
  if(failure == boost::geometry::failure_wrong_orientation)
  {
    // counter-clockwise, so first line is port
    m_swath_side = BoatSide::Port;
    RCLCPP_INFO_STREAM(logger_, "Port side");
  }
  else
  {
    // clockwise so stbd first
    m_swath_side = BoatSide::Stbd;
    RCLCPP_INFO_STREAM(logger_, "Starboard side");
  }
  m_swath_record.SetOutputSide(m_swath_side);
    
  if(!valid)
  {
    RCLCPP_WARN_STREAM(logger_, "Invalid polygon, trying to correct");
    boost::geometry::correct(m_op_region);
  }

  std::string reason;
  valid = boost::geometry::is_valid(m_op_region, reason);
  if(!valid)
    RCLCPP_WARN_STREAM(logger_, "Invalid polygon: " << reason);

  RCLCPP_INFO_STREAM(logger_, "Goal received: polygon with " << m_op_region.outer().size() << " vertices");

  RCLCPP_INFO_STREAM(logger_, "Initial line");
  for(int i = 0; i < m_survey_path.size(); i++)
      RCLCPP_INFO_STREAM(logger_, "    point: " << m_survey_path.get_vx(i) << ", " << m_survey_path.get_vy(i));

  m_swath_record.ResetLine();

  // Set the alignment lines and turn for the first line
  DetermineStartAndTurn(m_survey_path);
}




void SurveyPath::odomCallback(const nav_msgs::msg::Odometry::UniquePtr &odom_msg)
{
  // Held for the whole callback: it reads waypoint_distance_threshold_ and,
  // via CreateNewPath, the swath_overlap/max_bend_angle/lead-distance members
  // and m_swath_record — all mutated by the post-set apply callback in another
  // callback group. The lock serializes against that apply path. Helpers reached
  // from here (CreateNewPath, extendPathForLeadInOut) must NOT re-lock — the
  // mutex is non-recursive.
  std::lock_guard<std::mutex> lock(m_param_mutex);

  const auto& odom = *odom_msg;
  current_odom_ = odom;

  if(m_survey_path.size() < 2)
      return;

  BPoint current_position(odom.pose.pose.position.x, odom.pose.pose.position.y);

  if(m_state == transit)
  {
    auto distance_to_start = boost::geometry::distance(current_position, BPoint(m_survey_path.get_vx(0), m_survey_path.get_vy(0)));
    if(distance_to_start < waypoint_distance_threshold_)
    {
      RCLCPP_INFO_STREAM(logger_, "End of line, transit -> survey");
      m_state = survey;
      m_recording = true;
    }
  }
  else if(m_state == survey)
  {
    auto distance_to_end = boost::geometry::distance(current_position, BPoint(m_survey_path.get_vx(m_survey_path.size()-1), m_survey_path.get_vy(m_survey_path.size()-1)));
    if(distance_to_end < waypoint_distance_threshold_)
    {
      RCLCPP_INFO_STREAM(logger_, "End of line, survey -> transit");
      m_recording = false;
      CreateNewPath();
      m_state = transit;
    }
  }
}

void SurveyPath::pingCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ping)
{
  // Held for the whole callback: it mutates and reads m_swath_record
  // (AddRecord, SwathOuterPts), whose interval/min-swath thresholds the
  // post-set apply callback updates from another callback group. Serializes
  // against that apply path.
  std::lock_guard<std::mutex> lock(m_param_mutex);

  RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 1000, "Ping!" << " recording: " << m_recording << " state: " << (m_state==transit?"transit":"survey") << " x: " << current_odom_.pose.pose.position.x << " y: " << current_odom_.pose.pose.position.y);

  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*ping, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*ping, "z");

  float miny, maxy;

  std::vector<float> depths;

  if(iter_y != iter_y.end())
    miny = maxy = *iter_y;

  while(iter_y != iter_y.end() && iter_z != iter_z.end())
  {
    depths.push_back(*iter_z);
    miny = std::min(miny,*iter_y);
    maxy = std::max(maxy,*iter_y);
    ++iter_y;
    ++iter_z;
  }

  if(depths.empty())
      return;

    // TODO apply TF transform, following assumes standard MBES install with z down, x forward and y port
  m_swath_info["port"] = maxy;
  m_swath_info["stbd"] = -miny;

    // crude approximation of nadir depth
  m_swath_info["depth"] = depths[depths.size()/2];

  m_swath_info["x"] = current_odom_.pose.pose.position.x;
  m_swath_info["y"] = current_odom_.pose.pose.position.y;
  auto yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  m_swath_info["hdg"] = 90-(yaw*180.0/M_PI);

  if (m_recording) 
  {
    RCLCPP_DEBUG_STREAM(logger_, "Recording swath: port " << m_swath_info["port"] << " stbd " << m_swath_info["stbd"]
                            << " x " << m_swath_info["x"] << " y " << m_swath_info["y"]
                            << " hdg " << m_swath_info["hdg"] << " depth " << m_swath_info["depth"]);
    m_swath_record.AddRecord(m_swath_info["stbd"], m_swath_info["port"],
                              m_swath_info["x"], m_swath_info["y"], m_swath_info["hdg"],
                              m_swath_info["depth"]);


    XYSegList points = m_swath_record.SwathOuterPts(m_swath_side);
    if(points.size() > 0 && m_display_publisher)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = m_map_frame;
      marker.header.stamp = clock_->now();
      marker.ns = "manda_coverage_swath";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      for(int i = 0; i < points.size(); i++)
      {
          geometry_msgs::msg::Point p;
          p.x = points.get_vx(i);
          p.y = points.get_vy(i);
          marker.points.push_back(p);
      }
      marker.color.r = .3;
      marker.color.g = .4;
      marker.color.b = .5;
      marker.color.a = .5;

      marker.lifetime = rclcpp::Duration::from_seconds(5.0);

      m_display_publisher->publish(marker);

      path_marker_.header.frame_id = marker.header.frame_id;
      path_marker_.header.stamp = marker.header.stamp;
      path_marker_.ns = "manda_coverage_swath";
      path_marker_.id = 1;
      path_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker_.action = visualization_msgs::msg::Marker::ADD;
      path_marker_.scale.x = 1.5;
      path_marker_.scale.y = 1.5;
      path_marker_.scale.z = 1.5;
      path_marker_.color.r = .8;
      path_marker_.color.g = .2;
      path_marker_.color.b = .2;
      path_marker_.color.a = .8;
      path_marker_.lifetime = rclcpp::Duration::from_seconds(5.0);


    }
  }

}

bool SurveyPath::SwathOutsideRegion() 
{
  std::pair<XYPoint, XYPoint> swath_edges = m_swath_record.LastOuterPoints();
  BPoint port_edge(swath_edges.first.x(), swath_edges.first.y());
  BPoint stbd_edge(swath_edges.second.x(), swath_edges.second.y());

  auto outer_ring = m_op_region.outer();
  bool outside_region = !boost::geometry::within(port_edge, outer_ring);
  outside_region = outside_region && !boost::geometry::within(stbd_edge, outer_ring);

  return outside_region;
}


void SurveyPath::CreateNewPath()
{
  m_swath_record.SaveLast();
  if (m_swath_record.ValidRecord())
  {
    // TODO: Check for all swath widths being zero to end area
    // Build full coverage model at some point? Or do this in PathPlan...
    PathPlan planner = PathPlan(m_swath_record, m_swath_side, m_op_region,
                                m_swath_overlap, m_max_bend_angle, true);
    m_survey_path = planner.GenerateNextPath();
    if (m_survey_path.size() > 2) 
    {
        DetermineStartAndTurn(m_survey_path);
    }
    else 
    {
      m_state = idle;
      if(done_callback_)
      {
        done_callback_(true);
      }
    }
    m_swath_side = AdvanceSide(m_swath_side);
    m_swath_record.SetOutputSide(m_swath_side);
    m_swath_record.ResetLine();
  }
}

bool SurveyPath::DetermineStartAndTurn(XYSegList& next_pts) 
{
  extendPathForLeadInOut(next_pts);
  sendPath(next_pts);
  m_state = transit;
  m_recording = false;

  return true;
}

void SurveyPath::sendPath(XYSegList const &path)
{
  nav_msgs::msg::Path next_path;
  next_path.header.frame_id = m_map_frame;
  for(int i = 0; i < path.size(); i++)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = m_map_frame;
    pose.pose.position.x = path.get_vx(i);
    pose.pose.position.y = path.get_vy(i);

    next_path.poses.push_back(pose);
  }

  marine_nav_utilities::adjustPathOrientations(next_path.poses);

  m_line_number += 1;

  if(next_line_callback_)
  {
    next_line_callback_(next_path, m_line_number);
  }

  path_marker_.points.clear();
  for(int i = 0; i < path.size(); i++)
  {
      geometry_msgs::msg::Point p;
      p.x = path.get_vx(i);
      p.y = path.get_vy(i);
      path_marker_.points.push_back(p);
  }
}


BoatSide SurveyPath::AdvanceSide(BoatSide side)
{
  if (side == BoatSide::Stbd)
  {
    RCLCPP_INFO_STREAM(logger_, "Starboard to Port");
    return BoatSide::Port;
  } 
  else if (side == BoatSide::Port) 
  {
    RCLCPP_INFO_STREAM(logger_, "Port to Starboard");
    return BoatSide::Stbd;
  }
  return BoatSide::Unknown;
}

void SurveyPath::extendPathForLeadInOut(XYSegList& path)
{
  if(path.size() < 2)
    return;

  // Lead-in
  tf2::Vector3 start_vector(path.get_vx(0), path.get_vy(0), 0);
  tf2::Vector3 next_vector(path.get_vx(1), path.get_vy(1), 0);
  tf2::Vector3 direction_vector = next_vector - start_vector;
  direction_vector.normalize();

  start_vector -= direction_vector * lead_in_distance_;
  path.set_vx(0, start_vector.x());
  path.set_vy(0, start_vector.y());

  // Lead-out
  auto last_index = path.size()-1;
  auto next_to_last_index = last_index - 1;
  tf2::Vector3 last_position(path.get_vx(last_index), path.get_vy(last_index), 0);
  tf2::Vector3 next_to_last_position(path.get_vx(next_to_last_index), path.get_vy(next_to_last_index), 0);
  direction_vector = last_position - next_to_last_position;
  direction_vector.normalize();

  last_position += direction_vector * lead_out_distance_;
  path.set_vx(last_index, last_position.x());
  path.set_vy(last_index, last_position.y());
}

} // namespace manda_coverage

