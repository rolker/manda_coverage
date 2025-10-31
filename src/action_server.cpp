#include "manda_coverage/action_server.h"

namespace manda_coverage
{

  MandaCoverageActionServer::MandaCoverageActionServer(const rclcpp::NodeOptions & options)
 : nav2_util::LifecycleNode("manda_coverage_action_server", "", options)
{

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MandaCoverageActionServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring MandaCoverageActionServer...");

  survey_path_ = std::make_shared<SurveyPath>(*this);
  survey_path_->set_next_line_callback(std::bind(&MandaCoverageActionServer::next_line_callback, this, std::placeholders::_1, std::placeholders::_2));
  survey_path_->set_done_callback(std::bind(&MandaCoverageActionServer::done_callback, this, std::placeholders::_1));
  survey_path_->configure();

  action_server_ = rclcpp_action::create_server<ComputeSonarCoveragePath>(
    shared_from_this(),
    "compute_sonar_coverage_path",
    std::bind(&MandaCoverageActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MandaCoverageActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&MandaCoverageActionServer::handle_accepted, this, std::placeholders::_1)
  );

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MandaCoverageActionServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating MandaCoverageActionServer...");
  survey_path_->activate();
  createBond();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MandaCoverageActionServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating MandaCoverageActionServer...");
  survey_path_->deactivate();
  destroyBond();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MandaCoverageActionServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up MandaCoverageActionServer...");
  survey_path_->cleanup();
  survey_path_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MandaCoverageActionServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down MandaCoverageActionServer...");
  if (survey_path_) {
    survey_path_->deactivate();
    survey_path_->cleanup();
    survey_path_.reset();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse MandaCoverageActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ComputeSonarCoveragePath::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

} // namespace manda_coverage

rclcpp_action::CancelResponse MandaCoverageActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleComputeSonarCoveragePath> goal_handle)
{
  survey_path_->set_goal(geometry_msgs::msg::PolygonStamped()); // Clear the goal to stop the survey path from executing.
  current_goal_handle_.reset();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MandaCoverageActionServer::handle_accepted(const std::shared_ptr<GoalHandleComputeSonarCoveragePath> goal_handle)
{
  auto goal = goal_handle->get_goal();
  current_goal_handle_ = goal_handle;
  survey_path_->set_goal(goal->survey_area);
}

void MandaCoverageActionServer::next_line_callback(const nav_msgs::msg::Path& path, int line_number)
{
  if(!current_goal_handle_)
  {
    RCLCPP_WARN(get_logger(), "No active goal handle to send feedback to.");
    return;
  }
  auto feedback = std::make_shared<ComputeSonarCoveragePath::Feedback>();
  feedback->current_line = path;
  feedback->line_number = line_number;
  current_goal_handle_->publish_feedback(feedback);
}

void MandaCoverageActionServer::done_callback(bool success)
{
  if(!current_goal_handle_)
  {
    RCLCPP_WARN(get_logger(), "No active goal handle to send result to.");
    return;
  }
  auto result = std::make_shared<ComputeSonarCoveragePath::Result>();
  result->success = success;
  current_goal_handle_->succeed(result);
  current_goal_handle_.reset();
}

} // namespace manda_coverage
