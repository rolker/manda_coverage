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
#include <geographic_msgs/GeoPoseStamped.h>
#include "geographic_visualization_msgs/GeoVizItem.h"

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
  m_execute_path_plan{false}, m_transformations(m_node),
  m_action_server(m_node, "survey_area_action", false),
  m_path_follower_client(m_node, "path_follower_action"), m_autonomous_state(false)
{
    m_swath_record.SetOutputSide(m_swath_side);

    ros::Subscriber ping_sub = m_node.subscribe("/mbes_ping",10, &SurveyPath::pingCallback, this);
    ros::Subscriber depth_sub = m_node.subscribe("/depth",10, &SurveyPath::depthCallback, this);
    ros::Subscriber position_sub = m_node.subscribe("/position_map", 10, &SurveyPath::positionCallback, this);
    ros::Subscriber heading_sub = m_node.subscribe("/heading", 10, &SurveyPath::headingCallback, this);
    ros::Subscriber state_sub = m_node.subscribe("/project11/piloting_mode", 10, &SurveyPath::stateCallback, this);
    
    m_display_pub = m_node.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",5);

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
    m_swath_info["stbd"] = -miny;
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
        XYSegList points = m_swath_record.SwathOuterPts(m_swath_side);
        geographic_visualization_msgs::GeoVizItem vizItem;
        vizItem.id = "manda_coverage_swath";
        if(points.size() > 0)
        {
            while (!m_transformations.haveOrigin())
            {
                std::cerr << "SurveyPath::Iterate waiting for origin..." << std::endl;
                ros::Duration(0.5).sleep();
            }

            geographic_visualization_msgs::GeoVizPointList plist;
            plist.size = 2;
            for(int i = 0; i < points.size(); i++)
            {
                geometry_msgs::Point p;
                p.x = points.get_vx(i);
                p.y = points.get_vy(i);
                
                geographic_msgs::GeoPoint gp = m_transformations.map_to_wgs84(p);
                plist.points.push_back(gp);
            }
            plist.color.r = .3;
            plist.color.g = .4;
            plist.color.b = .5;
            plist.color.a = .5;
            vizItem.lines.push_back(plist);
        }
        m_display_pub.publish(vizItem);
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
    m_survey_path.clear();

    while (!m_transformations.haveOrigin())
        ros::Duration(0.5).sleep();
    
    for(auto point: goal->area)
    {
        geometry_msgs::Point p = m_transformations.wgs84_to_map(point);
        boost::geometry::append(m_op_region.outer(), BPoint(p.x,p.y));
        if(m_survey_path.size() < 2)
            m_survey_path.add_vertex(p.x,p.y);
    }
    boost::geometry::append(m_op_region.outer(),m_op_region.outer()[0]);

    boost::geometry::validity_failure_type failure;
    bool valid = boost::geometry::is_valid(m_op_region, failure);
    if(failure == boost::geometry::failure_wrong_orientation)
    {
        // counter-clockwise, so first line is port
        m_swath_side = BoatSide::Port;
    }
    else
    {
        // clockwise so stbd first
        m_swath_side = BoatSide::Stbd;
    }
    m_swath_record.SetOutputSide(m_swath_side);
    
    if(!valid)
    {
        std::cerr << "Trying to correct invalid polygon" << std::endl;
        boost::geometry::correct(m_op_region);
    }

    std::string reason;
    valid = boost::geometry::is_valid(m_op_region, reason);
    if(!valid)
        std::cerr << "Invalid polygon: " << reason << std::endl;
    

    // Set the alignment lines and turn for the first line
    DetermineStartAndTurn(m_survey_path);
    
    //m_recording = true;
    m_line_end = false;
}

void SurveyPath::preemptCallback()
{
    m_action_server.setPreempted();
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
    
    while (!m_transformations.haveOrigin())
    {
        std::cerr << "SurveyPath::sendPath waiting for origin..." << std::endl;
        ros::Duration(0.5).sleep();
    }

    for(int i = 0; i < path.size(); i++)
    {
        geometry_msgs::Point point;
        point.x = path.get_vx(i);
        point.y = path.get_vy(i);
        
        auto position = m_transformations.map_to_wgs84(point);

        geographic_msgs::GeoPoseStamped gps;
        gps.pose.position.latitude = position.latitude;
        gps.pose.position.longitude = position.longitude;
        goal.path.poses.push_back(gps);
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
