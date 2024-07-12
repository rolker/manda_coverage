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

//---------------------------------------------------------
// Constructor

SurveyPath::SurveyPath() :
    m_swath_record(10),
      m_action_server(m_node, "survey_area_action", false)
{
    m_swath_record.SetOutputSide(m_swath_side);

    ros::Subscriber ping_sub = m_node.subscribe("soundings",10, &SurveyPath::pingCallback, this);

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
    m_swath_info["port"] = -miny;
    m_swath_info["stbd"] = -maxy;

    // crude approximation of nader depth
    m_swath_info["depth"] = pc.points[pc.points.size()/2].z;

    Iterate();
}

void SurveyPath::odometryCallback(const nav_msgs::Odometry::ConstPtr &inmsg)
{
    m_swath_info["x"] = inmsg->pose.pose.position.x;
    m_swath_info["y"] = inmsg->pose.pose.position.y;
    m_swath_info["hdg"] = project11::quaternionToHeadingRadians(inmsg->pose.pose.orientation);

    // TODO: detect line end, maybe need to keep track of current line

    // void SurveyPath::PathFollowerDoneCallback(actionlib::SimpleClientGoalState const &state, path_follower::path_followerResult::ConstPtr const &result)
// {
//     m_line_end = true;
//     Iterate();
// }

}

void SurveyPath::Iterate()
{
    if (m_recording) 
    {
        m_swath_record.AddRecord(m_swath_info["stbd"], m_swath_info["port"],
                                 m_swath_info["x"], m_swath_info["y"], m_swath_info["hdg"],
                                 m_swath_info["depth"]);

        // TODO: replace visualization with some output (point arrays?) than can optionally visualized
        // XYSegList points = m_swath_record.SwathOuterPts(m_swath_side);
        // geographic_visualization_msgs::GeoVizItem vizItem;
        // vizItem.id = "manda_coverage_swath";
        // if(points.size() > 0)
        // {
        //     while (!m_transformations()->canTransform(m_map_frame, "earth", ros::Time(0), ros::Duration(0.5)))
        //     {
        //         std::cerr << "SurveyPath::Iterate waiting for origin..." << std::endl;
        //     }

        //     geographic_visualization_msgs::GeoVizPointList plist;
        //     plist.size = 2;
        //     for(int i = 0; i < points.size(); i++)
        //     {
        //         geometry_msgs::Point p;
        //         p.x = points.get_vx(i);
        //         p.y = points.get_vy(i);
                
        //         geographic_msgs::GeoPoint gp = m_transformations.map_to_wgs84(p);
        //         plist.points.push_back(gp);
        //     }
        //     plist.color.r = .3;
        //     plist.color.g = .4;
        //     plist.color.b = .5;
        //     plist.color.a = .5;
        //     vizItem.lines.push_back(plist);
        // }
        // m_display_pub.publish(vizItem);
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
    }
    else
    {
        // clockwise so stbd first
        m_swath_side = BoatSide::Stbd;
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
    project11_nav_msgs::multibeam_coverageFeedback feedback;
    for(int i = 0; i < path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = path.get_vx(i);
        pose.pose.position.y = path.get_vy(i);

        feedback.current_line.poses.push_back(pose);
    }

    m_action_server.publishFeedback(feedback);
    
    m_line_end = false;
}


BoatSide SurveyPath::AdvanceSide(BoatSide side)
{
    //std::cerr << "SurveyPath::AdvanceSide: ";
    if (side == BoatSide::Stbd)
    {
        //std::cerr << "stbd to port" << std::endl;
        return BoatSide::Port;
    } 
    else if (side == BoatSide::Port) 
    {
        //std::cerr << "port to stbd" << std::endl;
        return BoatSide::Stbd;
    }
    return BoatSide::Unknown;
}
