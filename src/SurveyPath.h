/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.h                                    */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#ifndef SurveyPath_HEADER
#define SurveyPath_HEADER

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//#include "manda_coverage/manda_coverageAction.h"
#include "project11_nav_msgs/multibeam_coverageAction.h"
#include "actionlib/server/simple_action_server.h"
#include <actionlib/client/simple_action_client.h>

#include <thread>
//#include "XYPoint.h"
#include "RecordSwath.h"
#include "PathPlan.h"
#include "project11/tf2_utils.h"


class SurveyPath
{
public:
    SurveyPath();
    ~SurveyPath() {};

protected:
    void Iterate();

    BoatSide AdvanceSide(BoatSide side);
    bool DetermineStartAndTurn(XYSegList& next_pts);
    void CreateNewPath();
    bool SwathOutsideRegion();
    
    void goalCallback();
    void preemptCallback();

    void pingCallback(const sensor_msgs::PointCloud2::ConstPtr &inmsg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &inmsg);

    void sendPath(XYSegList const &);

private: // Configuration variables
    BoatSide m_first_swath_side = BoatSide::Stbd;
    double m_swath_interval = 10;
    double m_alignment_line_len = 10;
    double m_turn_pt_offset = 15;
    bool m_remove_in_coverage = false;
    double m_swath_overlap = 0.2;
    double m_max_bend_angle = 60;
    std::string m_map_frame;

private: // State variables
    enum State {idle, transit, survey};
    State m_state = State::idle;
     
    //BoatSide m_next_swath_side;
    BoatSide m_swath_side = BoatSide::Stbd;
    bool m_line_end = false;
    bool m_line_begin = false;
    bool m_turn_reached = false;
    bool m_recording = false;
    BPolygon m_op_region;
    RecordSwath m_swath_record;
    std::map<std::string, double> m_swath_info;
    std::string m_posted_path_str;
    XYSegList m_survey_path;
    XYSegList m_raw_survey_path;
    XYPoint m_turn_pt;
    XYSegList m_alignment_line;

    ros::NodeHandle m_node;

    actionlib::SimpleActionServer<project11_nav_msgs::multibeam_coverageAction> m_action_server;
};

#endif
