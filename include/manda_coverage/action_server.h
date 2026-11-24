#ifndef MANDA_COVERAGE_ACTION_SERVER_H
#define MANDA_COVERAGE_ACTION_SERVER_H

#include "marine_nav_interfaces/action/compute_sonar_coverage_path.hpp"
#include "rclcpp/rclcpp.hpp"
// Nav2's lifecycle node moved from nav2_util (Jazzy) to nav2_ros_common
// (Rolling); the alias below names whichever one exists.
#if __has_include("nav2_ros_common/lifecycle_node.hpp")
#include "nav2_ros_common/lifecycle_node.hpp"
#else
#include "nav2_util/lifecycle_node.hpp"
#endif
#include "rclcpp_action/rclcpp_action.hpp"
#include "marine_control/control_server.hpp"
#include "manda_coverage/SurveyPath.h"

namespace manda_coverage
{ 

#if __has_include("nav2_ros_common/lifecycle_node.hpp")
using Nav2LifecycleNode = nav2::LifecycleNode;
#else
using Nav2LifecycleNode = nav2_util::LifecycleNode;
#endif

class MandaCoverageActionServer : public Nav2LifecycleNode
{
public:
  using ComputeSonarCoveragePath = marine_nav_interfaces::action::ComputeSonarCoveragePath;
  using GoalHandleComputeSonarCoveragePath = rclcpp_action::ServerGoalHandle<ComputeSonarCoveragePath>;

  explicit MandaCoverageActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MandaCoverageActionServer() = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_action::Server<ComputeSonarCoveragePath>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputeSonarCoveragePath::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputeSonarCoveragePath> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleComputeSonarCoveragePath> goal_handle);

  void next_line_callback(const nav_msgs::msg::Path& path, int line_number);
  void done_callback(bool success);

  std::shared_ptr<SurveyPath> survey_path_;
  std::shared_ptr<GoalHandleComputeSonarCoveragePath> current_goal_handle_;

  // Bridge-side operator control (ADR-0003). Constructed in on_activate(),
  // reset in every teardown path. The server is active as soon as it is
  // constructed, so lifecycle gating is achieved by construct-on-activate /
  // reset-on-teardown rather than by the server itself.
  std::unique_ptr<marine_control::ControlServer> control_server_;

};


} // namespace manda_coverage

#endif
