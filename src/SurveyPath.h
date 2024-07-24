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
    void navigationStateCallback(const std_msgs::String::ConstPtr &inmsg);

    void sendPath(XYSegList const &);

private: // Configuration variables
    BoatSide m_first_swath_side = BoatSide::Stbd;
    double m_swath_interval = 10;
    bool m_remove_in_coverage = false;
    double m_swath_overlap = 0.2;
    double m_max_bend_angle = 60;
    std::string m_map_frame;
    int m_line_number = 0;

private: // State variables
    enum State {idle, transit, survey};
    State m_state = State::idle;
     
    //BoatSide m_next_swath_side;
    BoatSide m_swath_side = BoatSide::Stbd;
    bool m_line_end = false;
    bool m_recording = false;
    BPolygon m_op_region;
    RecordSwath m_swath_record;
    std::map<std::string, double> m_swath_info;
    XYSegList m_survey_path;

    ros::NodeHandle m_node;

    actionlib::SimpleActionServer<project11_nav_msgs::multibeam_coverageAction> m_action_server;
    ros::Subscriber m_ping_subscription;
    ros::Subscriber m_odometry_subscription;
    ros::Subscriber m_navigation_state_subscription;

    ros::Publisher m_display_publisher;
};

#endif
