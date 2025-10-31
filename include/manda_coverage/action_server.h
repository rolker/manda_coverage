#ifndef MANDA_COVERAGE_ACTION_SERVER_H
#define MANDA_COVERAGE_ACTION_SERVER_H

#include "marine_nav_interfaces/action/compute_sonar_coverage_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "manda_coverage/SurveyPath.h"

namespace manda_coverage
{ 

class MandaCoverageActionServer : public nav2_util::LifecycleNode
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

};


} // namespace manda_coverage

#endif
