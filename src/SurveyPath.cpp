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

#include "ros/ros.h"
#include "project11_transformations/LatLongToMap.h"
#include "project11_transformations/MapToLatLong.h"
#include <geographic_msgs/GeoPoseStamped.h>

#define DEBUG true

typedef std::list<std::string > STRING_LIST;

//---------------------------------------------------------
// Constructor

SurveyPath::SurveyPath() : m_first_swath_side{BoatSide::Stbd},
  m_swath_interval{10}, m_alignment_line_len{10}, m_turn_pt_offset{15},
  m_remove_in_coverage{false}, m_swath_overlap{0.2}, m_line_end{false},
  m_line_begin{false}, m_turn_reached{false}, m_recording{false},
  m_swath_record(10), m_state(idle), m_swath_side{BoatSide::Stbd},
   m_max_bend_angle{60},
  m_execute_path_plan{false},
  m_action_server(m_node, "survey_area_action", false),
  m_path_follower_client(m_node, "path_follower_action"), m_autonomous_state(false)
{
    m_swath_record.SetOutputSide(m_swath_side);

    ros::Subscriber ping_sub = m_node.subscribe("/mbes_ping",10, &SurveyPath::pingCallback, this);
    ros::Subscriber depth_sub = m_node.subscribe("/depth",10, &SurveyPath::depthCallback, this);
    ros::Subscriber position_sub = m_node.subscribe("/position_map", 10, &SurveyPath::positionCallback, this);
    ros::Subscriber heading_sub = m_node.subscribe("/heading", 10, &SurveyPath::headingCallback, this);
    ros::Subscriber state_sub = m_node.subscribe("/project11/piloting_mode", 10, &SurveyPath::stateCallback, this);

    m_action_server.registerGoalCallback(boost::bind(&SurveyPath::goalCallback, this));
    m_action_server.registerPreemptCallback(boost::bind(&SurveyPath::preemptCallback, this));
    m_action_server.start();
    
    ROS_INFO("Waiting for path_follower action server to start.");
    m_path_follower_client.waitForServer();
    ROS_INFO("Action server started.");

    ros::spin();
}

void SurveyPath::pingCallback(const sensor_msgs::PointCloud::ConstPtr& inmsg)
{
    float miny, maxy;
    
    miny = maxy = inmsg->points[0].y;
    for (auto p: inmsg->points)
    {
        miny = std::min(miny,p.y);
        maxy = std::max(maxy,p.y);
    }
    m_swath_info["port"] = maxy;
    m_swath_info["stbd"] = miny;
    Iterate();
}

void SurveyPath::depthCallback(const std_msgs::Float32::ConstPtr& inmsg)
{
    m_swath_info["depth"] = inmsg->data;
}

void SurveyPath::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& inmsg)
{
    m_swath_info["x"] = inmsg->pose.position.x;
    m_swath_info["y"] = inmsg->pose.position.y;
}

void SurveyPath::headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
{
    m_swath_info["hdg"] = inmsg->orientation.heading;
}

void SurveyPath::stateCallback(const std_msgs::String::ConstPtr &inmsg)
{
    m_autonomous_state = inmsg->data == "autonomous";
}


void SurveyPath::Iterate()
{
    if (m_recording) 
    {
        m_swath_record.AddRecord(m_swath_info["stbd"], m_swath_info["port"],
                                 m_swath_info["x"], m_swath_info["y"], m_swath_info["hdg"],
                                 m_swath_info["depth"]);
    }
    if (m_line_end) 
    {
        if(m_state == transit)
        {
            m_line_end = false;
            sendPath(m_survey_path);
            m_state = survey;
            m_recording = true;
        }
        else if(m_state == survey)
        {
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
    auto goal = m_action_server.acceptNewGoal();
    
    m_desired_speed = goal->speed;

    m_op_region.clear();
    
    std::cerr << "SurveyPath::surveyAreaCallback: waiting for wgs84_to_map service..." << std::endl;
    ros::service::waitForService("wgs84_to_map");
    std::cerr << "done!" << std::endl;
    ros::ServiceClient client = m_node.serviceClient<project11_transformations::LatLongToMap>("wgs84_to_map");

    for(auto point: goal->area)
    {
        project11_transformations::LatLongToMap ll2map;
        ll2map.request.wgs84.position = point;
        if(client.call(ll2map))
        {
            boost::geometry::append(m_op_region.outer(), BPoint(ll2map.response.map.point.x,ll2map.response.map.point.y));
            std::cerr << ll2map.response.map.point.x << ", " << ll2map.response.map.point.y << std::endl;
        }
    }
    PostSurveyRegion();
    
    //m_recording = true;
    m_line_end = false;
}

void SurveyPath::preemptCallback()
{
    m_action_server.setPreempted();
}

void SurveyPath::PostSurveyRegion()
{
    #if DEBUG
    std::cout << "Posting Survey Area" << std::endl;
    #endif

    // Survey Region limits (currently only the outer ring)
    auto ext_ring = m_op_region.outer();
    XYSegList survey_limits;
    for (auto poly_vertex : ext_ring)
    {
        survey_limits.add_vertex(poly_vertex.x(), poly_vertex.y());
    }
    //   Notify("VIEW_SEGLIST", survey_limits.get_spec_pts(2) + ",label=op_region," +
    //     "label_color=red,edge_color=red,vertex_color=red,edge_size=2", MOOSTime());

  // Set the first path of the survey
    if (m_survey_path.size() < 2)
    {
        m_survey_path.clear();
        m_survey_path.add_vertex(survey_limits.get_vx(0), survey_limits.get_vy(0));
        m_survey_path.add_vertex(survey_limits.get_vx(1), survey_limits.get_vy(1));
    }

    // Set the alignment lines and turn for the first line
    DetermineStartAndTurn(m_survey_path);
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
            m_posted_path_str = m_survey_path.get_spec_pts(2);  //2 decimal precision
            DetermineStartAndTurn(m_survey_path);
        }
        else 
        {
        }
        m_swath_side = AdvanceSide(m_swath_side);
        m_swath_record.SetOutputSide(m_swath_side);
        m_swath_record.ResetLine();
        m_raw_survey_path = planner.GetRawPath();
    }
}

bool SurveyPath::DetermineStartAndTurn(XYSegList& next_pts) 
{
    std::size_t pts_len = next_pts.size();

  // The turn point, extended from the end of the path
    auto end_x = next_pts.get_vx(pts_len-1);
    auto end_y = next_pts.get_vy(pts_len-1);
    EPoint end_heading(end_x - next_pts.get_vx(pts_len-2),
                       end_y - next_pts.get_vy(pts_len-2));
    end_heading.normalize();
    end_heading *= m_turn_pt_offset;
    m_turn_pt = XYPoint(end_x + end_heading.x(), end_y + end_heading.y());
    m_turn_pt.set_spec_digits(2);

    // The alignment line, added to the beginning of the path
    EPoint start_heading(next_pts.get_vx(0) - next_pts.get_vx(1),
                         next_pts.get_vy(0) - next_pts.get_vy(1));
    start_heading.normalize();
    start_heading *= m_alignment_line_len;
    m_alignment_line.clear();
    m_alignment_line.add_vertex(next_pts.get_vx(0) + start_heading.x(),
                                next_pts.get_vy(0) + start_heading.y());
    m_alignment_line.add_vertex(next_pts.get_vx(0), next_pts.get_vy(0));

    XYSegList to_start_path;
    to_start_path.add_vertex( m_swath_info["x"], m_swath_info["y"]);
    to_start_path.add_vertex(m_alignment_line.get_vx(0), m_alignment_line.get_vy(0));
  
    //SetMOOSVar("ToStartPath", "points=" + to_start_path.get_spec_pts(2), MOOSTime());
    sendPath(to_start_path);
    m_state = transit;

    return true;
}

void SurveyPath::sendPath(XYSegList const &path)
{
    path_follower::path_followerGoal goal;
    goal.speed = m_desired_speed;
    std::cerr << "SurveyPath::sendPath: waiting for map_to_wgs84..." << std::endl;
    ros::service::waitForService("map_to_wgs84");
    std::cerr << "done!" << std::endl;
    ros::ServiceClient client = m_node.serviceClient<project11_transformations::MapToLatLong>("map_to_wgs84");
    
    for(int i = 0; i < path.size(); i++)
    {
        project11_transformations::MapToLatLong map2ll;
        map2ll.request.map.point.x = path.get_vx(i);
        map2ll.request.map.point.y = path.get_vy(i);
        if(client.call(map2ll))
        {
            geographic_msgs::GeoPoseStamped gps;
            gps.pose.position.latitude = map2ll.response.wgs84.position.latitude;
            gps.pose.position.longitude = map2ll.response.wgs84.position.longitude;
            goal.path.poses.push_back(gps);
        }
    }
    
    m_path_follower_client.sendGoal(goal, boost::bind(&SurveyPath::PathFollowerDoneCallback, this, _1, _2));
    m_line_end = false;
}

void SurveyPath::PathFollowerDoneCallback(actionlib::SimpleClientGoalState const &state, path_follower::path_followerResult::ConstPtr const &result)
{
    m_line_end = true;
    Iterate();
}

BoatSide SurveyPath::AdvanceSide(BoatSide side)
{
    std::cerr << "SurveyPath::AdvanceSide: ";
    if (side == BoatSide::Stbd)
    {
        std::cerr << "stbd to port" << std::endl;
        return BoatSide::Port;
    } 
    else if (side == BoatSide::Port) 
    {
        std::cerr << "port to stbd" << std::endl;
        return BoatSide::Stbd;
    }
    return BoatSide::Unknown;
}
