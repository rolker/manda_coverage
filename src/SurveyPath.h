/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.h                                    */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#ifndef SurveyPath_HEADER
#define SurveyPath_HEADER

#include <ros/ros.h>
#include <geographic_msgs/GeoPath.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <marine_msgs/NavEulerStamped.h>
#include "manda_coverage/manda_coverageAction.h"
#include "actionlib/server/simple_action_server.h"
#include <actionlib/client/simple_action_client.h>
#include <path_follower/path_followerAction.h>
#include <thread>
#include "XYPoint.h"
#include "RecordSwath.h"
#include "PathPlan.h"
#include "project11_transformations/local_services.h"


class SurveyPath
{
public:
    SurveyPath();
    ~SurveyPath() {};

protected:
    void Iterate();

    BoatSide AdvanceSide(BoatSide side);
    void PostSurveyRegion();
    bool DetermineStartAndTurn(XYSegList& next_pts);
    void CreateNewPath();
    bool SwathOutsideRegion();
    
    void goalCallback();
    void preemptCallback();
    void pingCallback(const sensor_msgs::PointCloud::ConstPtr &inmsg);
    void depthCallback(const std_msgs::Float32::ConstPtr &inmsg);
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg);
    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr &inmsg);
    void stateCallback(const std_msgs::String::ConstPtr &inmag);

    void sendPath(XYSegList const &);
    void PathFollowerDoneCallback(actionlib::SimpleClientGoalState const &state, path_follower::path_followerResult::ConstPtr const &result);

private: // Configuration variables
    BoatSide m_first_swath_side;
    double m_swath_interval;
    double m_alignment_line_len;
    double m_turn_pt_offset;
    bool m_remove_in_coverage;
    double m_swath_overlap;
    double m_max_bend_angle;

private: // State variables
    enum State {idle, transit, survey};
    State m_state;
     
    //BoatSide m_next_swath_side;
    BoatSide m_swath_side;
    bool m_line_end;
    bool m_line_begin;
    bool m_turn_reached;
    bool m_recording;
    BPolygon m_op_region;
    RecordSwath m_swath_record;
    std::map<std::string, double> m_swath_info;
    std::string m_posted_path_str;
    XYSegList m_survey_path;
    XYSegList m_raw_survey_path;
    XYPoint m_turn_pt;
    bool m_execute_path_plan;
    XYSegList m_alignment_line;

    ros::NodeHandle m_node;
    ros::Publisher m_display_pub;

    project11::Transformations m_transformations;
    actionlib::SimpleActionServer<manda_coverage::manda_coverageAction> m_action_server;
    
    actionlib::SimpleActionClient<path_follower::path_followerAction> m_path_follower_client;
    
    double m_desired_speed;
    bool m_autonomous_state;

};

#endif
