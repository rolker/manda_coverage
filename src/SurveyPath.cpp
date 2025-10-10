/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.cpp                                  */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#include <iterator>
#include <regex>
#include "MBUtils.h"
//#include "ACTable.h"
#include "AngleUtils.h"
#include "XYFormatUtilsSegl.h"
#include "RecordSwath.h"
#include "PathPlan.h"
#include "SurveyPath.h"
#include <boost/algorithm/string.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"

//---------------------------------------------------------
// Constructor

SurveyPath::SurveyPath(const rclcpp::NodeOptions & options) :
  nav2_util::LifecycleNode("manda_coverage", "", options),
    m_swath_record(10)
      //m_action_server(m_node, "survey_area_action", false)
{

}

nav2_util::CallbackReturn
SurveyPath::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  m_swath_side = BoatSide::Stbd;
  m_swath_record.SetOutputSide(m_swath_side);

  m_line_end = false;
  m_recording = false;

  nav2_util::declare_parameter_if_not_declared(
    node, "soundings_topic", rclcpp::ParameterValue("soundings"));

  std::string soundings_topic = get_parameter("soundings_topic").as_string();

  m_ping_subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
    soundings_topic, rclcpp::SensorDataQoS(), std::bind(&SurveyPath::pingCallback, this, std::placeholders::_1));

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);

  nav2_util::declare_parameter_if_not_declared(
    node, "navigation_state_topic", rclcpp::ParameterValue("navigation_state"));

  std::string navigation_state_topic = get_parameter("navigation_state_topic").as_string();

  m_navigation_state_subscription = create_subscription<std_msgs::msg::String>(
    navigation_state_topic, 10, std::bind(&SurveyPath::navigationStateCallback, this, std::placeholders::_1));

  nav2_util::declare_parameter_if_not_declared(
    node, "display_topic", rclcpp::ParameterValue(""));
  
  std::string display_topic = get_parameter("display_topic").as_string();
  if(display_topic != "")
    m_display_publisher = create_publisher<visualization_msgs::msg::Marker>(display_topic, 10);
  else
    m_display_publisher.reset();

  double action_server_result_timeout = 10.0;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "survey_area_action",
    std::bind(&SurveyPath::goalCallback, this),
    nullptr,
    std::chrono::milliseconds(500),
    false, server_options);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn SurveyPath::on_activate(const rclcpp_lifecycle::State & state)
{
  action_server_->activate();
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn SurveyPath::on_deactivate(const rclcpp_lifecycle::State & state)
{
  action_server_->deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn SurveyPath::on_cleanup(const rclcpp_lifecycle::State & state)
{
  action_server_.reset();
  m_ping_subscription.reset();
  m_navigation_state_subscription.reset();
  m_display_publisher.reset();
  odom_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

void SurveyPath::pingCallback(const sensor_msgs::msg::PointCloud2::UniquePtr& inmsg)
{

  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*inmsg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*inmsg, "z");

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
    // TODO apply TF transform, following assumes standard MBES install with z down, x forward and y port
  m_swath_info["port"] = maxy;
  m_swath_info["stbd"] = -miny;

    // crude approximation of nadir depth
  m_swath_info["depth"] = depths[depths.size()/2];

  auto odom = odom_sub_->getTwist();
  m_swath_info["x"] = odom.x;
  m_swath_info["y"] = odom.y;
  m_swath_info["hdg"] = 90-(odom.theta*180.0/M_PI);


  Iterate();
}

void SurveyPath::navigationStateCallback(const std_msgs::msg::String::UniquePtr &inmsg)
{
    switch(m_state)
    {
        case survey:
            if(inmsg->data != "survey_line")
            {
                m_line_end = true;
                Iterate();
            }
            break;
        case transit:
            if(inmsg->data == "survey_line")
            {
                m_line_end = true;
                Iterate();
            }
            break;
        case idle:
            // do nothing
            break;
    }

}

void SurveyPath::Iterate()
{
    if (m_recording) 
    {
        m_swath_record.AddRecord(m_swath_info["stbd"], m_swath_info["port"],
                                 m_swath_info["x"], m_swath_info["y"], m_swath_info["hdg"],
                                 m_swath_info["depth"]);


        XYSegList points = m_swath_record.SwathOuterPts(m_swath_side);
        if(points.size() > 0 && m_display_publisher)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = m_map_frame;
            marker.header.stamp = get_clock()->now();
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
        }
    }
    if (m_line_end) 
    {
        if(m_state == transit)
        {
          RCLCPP_INFO_STREAM(get_logger(), "End of line, transit -> survey");
            m_line_end = false;
            //sendPath(m_survey_path);
            m_state = survey;
            m_recording = true;
        }
        else if(m_state == survey)
        {
            RCLCPP_INFO_STREAM(get_logger(), "End of line, survey -> transit");
            m_recording = false;
            CreateNewPath();
            m_line_end = false;
            m_state = transit;
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

void SurveyPath::goalCallback()
{
  RCLCPP_INFO_STREAM(get_logger(), "Goal received");
  auto goal = action_server_->get_current_goal();
    
  m_map_frame = goal->survey_area.header.frame_id;
  RCLCPP_INFO_STREAM(get_logger(), "map frame: " << m_map_frame);

  m_op_region.clear();
  m_survey_path.clear();

  for(auto point: goal->survey_area.polygon.points)
  {
      boost::geometry::append(m_op_region.outer(), BPoint(point.x, point.y));
      if(m_survey_path.size() < 2)
          m_survey_path.add_vertex(point.x,point.y);
  }
  boost::geometry::append(m_op_region.outer(),m_op_region.outer()[0]);

  boost::geometry::validity_failure_type failure;
  bool valid = boost::geometry::is_valid(m_op_region, failure);
  if(failure == boost::geometry::failure_wrong_orientation)
  {
      // counter-clockwise, so first line is port
      m_swath_side = BoatSide::Port;
      RCLCPP_INFO_STREAM(get_logger(), "Port side");
  }
  else
  {
      // clockwise so stbd first
      m_swath_side = BoatSide::Stbd;
      RCLCPP_INFO_STREAM(get_logger(), "Starboard side");
  }
  m_swath_record.SetOutputSide(m_swath_side);
    
  if(!valid)
  {
      //std::cerr << "Trying to correct invalid polygon" << std::endl;
      boost::geometry::correct(m_op_region);
  }

  std::string reason;
  valid = boost::geometry::is_valid(m_op_region, reason);
  if(!valid)
    RCLCPP_WARN_STREAM(get_logger(), "Invalid polygon: " << reason);

  RCLCPP_INFO_STREAM(get_logger(), "Goal received: polygon with " << m_op_region.outer().size() << " vertices");

  RCLCPP_INFO_STREAM(get_logger(), "Initial line");
  for(int i = 0; i < m_survey_path.size(); i++)
      RCLCPP_INFO_STREAM(get_logger(), "    point: " << m_survey_path.get_vx(i) << ", " << m_survey_path.get_vy(i));

  m_swath_record.ResetLine();

  // Set the alignment lines and turn for the first line
  DetermineStartAndTurn(m_survey_path);
    
  //m_recording = true;
  m_line_end = false;

  rclcpp::Rate r(100);
  while(rclcpp::ok())
  {
    if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
    }

    if (action_server_->is_cancel_requested())
    {
      action_server_->terminate_all();
      RCLCPP_INFO(get_logger(), "Goal cancelled");
      return;
    }
    r.sleep();
  }

  action_server_->succeeded_current();

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
            auto result = std::make_shared<project11_nav_msgs::action::MultibeamCoverage_Result>();
            result->success = true;
            action_server_->succeeded_current(result);
        }
        m_swath_side = AdvanceSide(m_swath_side);
        m_swath_record.SetOutputSide(m_swath_side);
        m_swath_record.ResetLine();
    }
}

bool SurveyPath::DetermineStartAndTurn(XYSegList& next_pts) 
{
    sendPath(next_pts);
    m_state = transit;
    m_recording = false;

    return true;
}

void SurveyPath::sendPath(XYSegList const &path)
{
  RCLCPP_INFO_STREAM(get_logger(), "Sending path:");

  auto feedback = std::make_shared<project11_nav_msgs::action::MultibeamCoverage_Feedback>();
  for(int i = 0; i < path.size(); i++)
  {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = m_map_frame;
      pose.pose.position.x = path.get_vx(i);
      pose.pose.position.y = path.get_vy(i);

      feedback->current_line.poses.push_back(pose);
      RCLCPP_INFO_STREAM(get_logger(), "  Point: " << to_yaml(pose.pose.position));
  }
  m_line_number += 1;
  feedback->line_number = m_line_number;

  action_server_->publish_feedback(feedback);
    
  m_line_end = false;
}


BoatSide SurveyPath::AdvanceSide(BoatSide side)
{
  if (side == BoatSide::Stbd)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Starboard to Port");
    return BoatSide::Port;
  } 
  else if (side == BoatSide::Port) 
  {
    RCLCPP_INFO_STREAM(get_logger(), "Port to Starboard");
    return BoatSide::Stbd;
  }
  return BoatSide::Unknown;
}
