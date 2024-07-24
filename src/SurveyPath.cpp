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
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

//---------------------------------------------------------
// Constructor

SurveyPath::SurveyPath() :
    m_swath_record(10),
      m_action_server(m_node, "survey_area_action", false)
{
    m_swath_record.SetOutputSide(m_swath_side);

    std::string soundings_topic = ros::param::param<std::string>("~soundings_topic", "soundings");
    m_ping_subscription = m_node.subscribe(soundings_topic, 10, &SurveyPath::pingCallback, this);
    m_odometry_subscription = m_node.subscribe("odom", 10, &SurveyPath::odometryCallback, this);
    m_navigation_state_subscription = m_node.subscribe("navigator/navigation_state", 10, &SurveyPath::navigationStateCallback, this);
    m_display_publisher = ros::NodeHandle("~").advertise<visualization_msgs::Marker>("display", 10);

    m_action_server.registerGoalCallback(boost::bind(&SurveyPath::goalCallback, this));
    m_action_server.registerPreemptCallback(boost::bind(&SurveyPath::preemptCallback, this));
    m_action_server.start();
    
}

void SurveyPath::pingCallback(const sensor_msgs::PointCloud2::ConstPtr& inmsg)
{
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(*inmsg, pc);

    float miny, maxy;
    
    miny = maxy = pc.points[0].y;
    for (auto p: pc.points)
    {
        miny = std::min(miny,p.y);
        maxy = std::max(maxy,p.y);
    }
    // TODO apply TF transform, following assumes standard MBES install with z down, x forward and y port
    m_swath_info["port"] = maxy;
    m_swath_info["stbd"] = -miny;

    // crude approximation of nader depth
    m_swath_info["depth"] = pc.points[pc.points.size()/2].z;

    Iterate();
}

void SurveyPath::odometryCallback(const nav_msgs::Odometry::ConstPtr &inmsg)
{
    m_swath_info["x"] = inmsg->pose.pose.position.x;
    m_swath_info["y"] = inmsg->pose.pose.position.y;
    m_swath_info["hdg"] = project11::quaternionToHeadingDegrees(inmsg->pose.pose.orientation);
}

void SurveyPath::navigationStateCallback(const std_msgs::String::ConstPtr &inmsg)
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
        if(points.size() > 0)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = m_map_frame;
            marker.header.stamp = ros::Time::now();
            marker.ns = "manda_coverage_swath";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            for(int i = 0; i < points.size(); i++)
            {
                geometry_msgs::Point p;
                p.x = points.get_vx(i);
                p.y = points.get_vy(i);
                marker.points.push_back(p);
            }
            marker.color.r = .3;
            marker.color.g = .4;
            marker.color.b = .5;
            marker.color.a = .5;

            marker.lifetime = ros::Duration(5.0);

            m_display_publisher.publish(marker);
        }
    }
    if (m_line_end) 
    {
        if(m_state == transit)
        {
            ROS_INFO_STREAM("End of line, transit -> survey");
            m_line_end = false;
            //sendPath(m_survey_path);
            m_state = survey;
            m_recording = true;
        }
        else if(m_state == survey)
        {
            ROS_INFO_STREAM("End of line, survey -> transit");
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

    ROS_INFO_STREAM("Goal received");

    auto goal = m_action_server.acceptNewGoal();
    
    m_map_frame = goal->survey_area.header.frame_id;
    ROS_INFO_STREAM("map frame: " << m_map_frame);

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
        ROS_INFO_STREAM("Port side");
    }
    else
    {
        // clockwise so stbd first
        m_swath_side = BoatSide::Stbd;
        ROS_INFO_STREAM("Starboard side");
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
        ROS_WARN_STREAM("Invalid polygon: " << reason);

    ROS_INFO_STREAM("Goal received: polygon with " << m_op_region.outer().size() << " vertices");

    ROS_INFO_STREAM("Initial line");
    for(int i = 0; i < m_survey_path.size(); i++)
        ROS_INFO_STREAM("    point: " << m_survey_path.get_vx(i) << ", " << m_survey_path.get_vy(i));
    
    m_swath_record.ResetLine();

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
            DetermineStartAndTurn(m_survey_path);
        }
        else 
        {
            m_state = idle;
            project11_nav_msgs::multibeam_coverageResult result;
            result.success = true;
            m_action_server.setSucceeded(result);
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
    ROS_INFO_STREAM("Sending path:");
    project11_nav_msgs::multibeam_coverageFeedback feedback;
    for(int i = 0; i < path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = m_map_frame;
        pose.pose.position.x = path.get_vx(i);
        pose.pose.position.y = path.get_vy(i);

        feedback.current_line.poses.push_back(pose);
        ROS_INFO_STREAM("  Point: " << pose.pose.position);
    }
    m_line_number += 1;
    feedback.line_number = m_line_number;

    m_action_server.publishFeedback(feedback);
    
    m_line_end = false;
}


BoatSide SurveyPath::AdvanceSide(BoatSide side)
{
    if (side == BoatSide::Stbd)
    {
        ROS_INFO_STREAM("Starboard to Port");
        return BoatSide::Port;
    } 
    else if (side == BoatSide::Port) 
    {
        ROS_INFO_STREAM("Port to Starboard");
        return BoatSide::Stbd;
    }
    return BoatSide::Unknown;
}
