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

  // Adopt the marine_control ControlServer so the seven coverage tuning knobs are
  // visible to and settable from the operator station over the udp_bridge
  // (ROS 2 parameters are not bridgeable; ADR-0003). The server is active the
  // moment it is constructed, so we gate it on the lifecycle by constructing it
  // here and resetting it in every teardown path (on_deactivate/on_cleanup/
  // on_shutdown). The parameters were declared by survey_path_->configure() in
  // on_configure(), so bind_parameter() finds them.
  //
  // Deadlock analysis (issue #5 m_param_mutex + ControlServer): an inbound
  // ControlValue triggers ControlServer::on_change -> node->set_parameter, which
  // synchronously fires the post-set apply callback that takes m_param_mutex.
  // This chain cannot deadlock: on_change runs in the ControlServer's own
  // dedicated mutually-exclusive callback group (control_server.hpp threading
  // contract), while the post-set apply runs in the parameter-service group.
  // Neither re-enters the other -- the apply callback only copies validated
  // values into cached members / RecordSwath and never calls set_parameter or
  // touches the ControlServer -- so there is no lock-order inversion and
  // m_param_mutex is the only lock held across the apply.
  marine_control::ControlServerOptions opts;
  opts.device_name = "Manda Coverage";
  control_server_ = std::make_unique<marine_control::ControlServer>(this, opts);
  control_server_->bind_parameter("swath_overlap",               "fraction", "Coverage");
  control_server_->bind_parameter("max_bend_angle",              "deg",      "Coverage");
  control_server_->bind_parameter("swath_record_interval",       "m",        "Coverage");
  control_server_->bind_parameter("min_allowable_swath",         "m",        "Coverage");
  control_server_->bind_parameter("waypoint_distance_threshold", "m",        "Coverage");
  control_server_->bind_parameter("lead_in_distance",            "m",        "Coverage");
  control_server_->bind_parameter("lead_out_distance",           "m",        "Coverage");

  createBond();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MandaCoverageActionServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating MandaCoverageActionServer...");
  // Tear down the ControlServer while the node is being deactivated (its timer
  // and change subscription stop). reset() is idempotent on a null pointer.
  control_server_.reset();
  survey_path_->deactivate();
  destroyBond();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MandaCoverageActionServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up MandaCoverageActionServer...");
  // Safety net: if a configured-but-never-activated node is cleaned up, this is
  // a no-op; if cleanup follows deactivate, control_server_ is already null.
  control_server_.reset();
  survey_path_->cleanup();
  survey_path_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MandaCoverageActionServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down MandaCoverageActionServer...");
  // A direct active->shutdown skips on_deactivate, so reset here too to avoid
  // the node destructor tearing the server down while it may still be spinning
  // (control_server.hpp lifecycle note). Idempotent if already reset.
  control_server_.reset();
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
