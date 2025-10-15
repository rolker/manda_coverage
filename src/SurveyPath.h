/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.h                                    */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#ifndef SurveyPath_HEADER
#define SurveyPath_HEADER

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"

#include "project11_nav_msgs/action/multibeam_coverage.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <thread>
#include "RecordSwath.h"
#include "PathPlan.h"

class SurveyPath: public nav2_util::LifecycleNode
{
public:
  using Action = project11_nav_msgs::action::MultibeamCoverage;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  explicit SurveyPath(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SurveyPath() = default;

protected:
  void Iterate();

  BoatSide AdvanceSide(BoatSide side);
  bool DetermineStartAndTurn(XYSegList& next_pts);
  void CreateNewPath();
  bool SwathOutsideRegion();
  
  void goalCallback();

  void pingCallback(const sensor_msgs::msg::PointCloud2::UniquePtr &inmsg);
  void odomCallback(const nav_msgs::msg::Odometry::UniquePtr &odom_msg);

  void sendPath(XYSegList const &);

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  
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
  bool in_polygon_ = false;
  BPolygon m_op_region;
  RecordSwath m_swath_record;
  std::map<std::string, double> m_swath_info;
  XYSegList m_survey_path;

  std::unique_ptr<ActionServer> action_server_;

  nav_msgs::msg::Odometry m_current_odom;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_ping_subscription;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_display_publisher;
  visualization_msgs::msg::Marker path_marker_;
};

#endif
