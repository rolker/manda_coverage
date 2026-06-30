/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: MIT                                             */
/*    FILE: main.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include "manda_coverage/action_server.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<manda_coverage::MandaCoverageActionServer>();
  // SingleThreadedExecutor (not multi-threaded): the marine_control ControlServer
  // adopted in on_activate() requires that its callbacks never run concurrently
  // with the lifecycle transitions that construct/bind and reset it
  // (control_server.hpp: "bind before spinning / destroy only when not spinning").
  // A single executor thread serializes every callback it dispatches — the
  // lifecycle-manager-driven transitions, the server's heartbeat timer and change
  // subscription, and the SurveyPath planning callbacks — so on the normal
  // (lifecycle-manager-driven) transition path the bindings_ bind race and the
  // timer/sub-teardown-vs-heartbeat race cannot occur. (SurveyPath's m_param_mutex
  // is retained defensively; see SurveyPath.h.)
  //
  // This does NOT fully eliminate the teardown race: nav2_util's LifecycleNode
  // registers an rcl pre-shutdown callback that drives deactivate()/cleanup() ->
  // control_server_.reset() on the signal-handler thread (not the executor
  // thread) during SIGINT, so a one-shot teardown-vs-heartbeat window remains on
  // the SIGINT-while-active path at process exit. The general fix lives in
  // marine_control (inject an external callback group / lock bindings_):
  // rolker/marine_control#12.
  //
  // Responsiveness tradeoff (deliberate, round-1 review): one thread means a long
  // planning callback (odom/ping -> CreateNewPath -> PathPlan) blocks the
  // ControlServer heartbeat/change handling and the action handling until it
  // returns. Acceptable for this lightweight planner; revisit (with the
  // marine_control#12 fix) if planning latency starts to starve operator control.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node->get_node_base_interface());

  executor->spin();
  rclcpp::shutdown();
  return 0;
}

