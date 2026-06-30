/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.h                                    */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#ifndef SurveyPath_HEADER
#define SurveyPath_HEADER

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "manda_coverage/RecordSwath.h"
#include "manda_coverage/PathPlan.h"

namespace manda_coverage
{ 

using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
  rclcpp::node_interfaces::NodeBaseInterface,
  rclcpp::node_interfaces::NodeClockInterface,
  rclcpp::node_interfaces::NodeLoggingInterface,
  rclcpp::node_interfaces::NodeParametersInterface,
  rclcpp::node_interfaces::NodeTopicsInterface
>;


class SurveyPath
{
public:
  using PositionedPing = std::pair<std::shared_ptr<sensor_msgs::msg::PointCloud2>,
                       nav_msgs::msg::Odometry>;

  explicit SurveyPath(NodeInterfaces node_interfaces);
  ~SurveyPath() = default;

  void configure();
  void activate();
  void deactivate();
  void cleanup();

  void set_next_line_callback(std::function<void(const nav_msgs::msg::Path&, int)> next_line_callback);
  void set_done_callback(std::function<void(bool)> done_callback);

  void set_goal(const geometry_msgs::msg::PolygonStamped &goal);

  // Accessors for the live-settable coverage-density parameters. Used by tests
  // to verify that a successful parameter set propagates into cached members
  // and live RecordSwath state.
  double swath_overlap() const { return m_swath_overlap; }
  double max_bend_angle() const { return m_max_bend_angle; }
  double swath_record_interval() const { return m_swath_record.IntervalDist(); }
  double min_allowable_swath() const { return m_swath_record.GetMinAllowableSwath(); }
  double waypoint_distance_threshold() const { return waypoint_distance_threshold_; }
  double lead_in_distance() const { return lead_in_distance_; }
  double lead_out_distance() const { return lead_out_distance_; }

private:

  BoatSide AdvanceSide(BoatSide side);
  bool DetermineStartAndTurn(XYSegList& next_pts);
  void CreateNewPath();
  bool SwathOutsideRegion();
  

  void pingCallback(const sensor_msgs::msg::PointCloud2::SharedPtr inmsg);
  void odomCallback(const nav_msgs::msg::Odometry::UniquePtr &odom_msg);


  void sendPath(XYSegList const &);

  void extendPathForLeadInOut(XYSegList& path);
  
// Configuration variables
  BoatSide m_first_swath_side = BoatSide::Stbd;
  bool m_remove_in_coverage = false;
  double m_swath_overlap = 0.2;
  double m_max_bend_angle = 60;
  std::string m_map_frame;
  int m_line_number = 0;

// State variables
  enum State {idle, transit, survey};
  State m_state = State::idle;
      
  //BoatSide m_next_swath_side;
  BoatSide m_swath_side = BoatSide::Stbd;

  bool m_recording = false;
  BPolygon m_op_region;
  RecordSwath m_swath_record;
  std::map<std::string, double> m_swath_info;
  XYSegList m_survey_path;

  /// Distance threshold to consider a waypoint reached
  double waypoint_distance_threshold_ = 4.0;

  /// Lead-in distance in meters to add to the beginning of a survey line
  double lead_in_distance_ = 15.0;

  /// Lead-out distance in meters to add to the end of a survey line
  double lead_out_distance_ = 5.0;

  nav_msgs::msg::Odometry current_odom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_ping_subscription;
  

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_display_publisher;
  visualization_msgs::msg::Marker path_marker_;

  NodeInterfaces node_interfaces_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Parameter-callback handle. The post-set callback applies committed values
  // to cached members and live RecordSwath state; it is reset in cleanup() to
  // unregister on teardown. Type/range rejection is enforced by the parameter
  // descriptors (rclcpp validates before the set commits), so no on-set
  // validation callback is registered.
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_param_callback_handle_;

  // Serializes the post-set apply path (param-service callback group) against
  // the planning readers — pingCallback/odomCallback and the action-server
  // set_goal — which run in other callback group(s) concurrently with the apply
  // under a MultiThreadedExecutor. (ping and odom share one MutuallyExclusive
  // group, so the executor already serializes them with each other; this mutex
  // is about the param-apply-vs-planning races.) Guards the cached tuning
  // members (m_swath_overlap, m_max_bend_angle, the distance members) and
  // m_swath_record's setter-mutated state.
  std::mutex m_param_mutex;

  std::function<void(bool)> done_callback_;
  std::function<void(const nav_msgs::msg::Path&, int)> next_line_callback_;
};

} // namespace manda_coverage

#endif
