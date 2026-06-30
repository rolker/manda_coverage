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
  // A single executor thread serializes every callback — the lifecycle
  // transitions, the server's heartbeat timer and change subscription, and the
  // SurveyPath planning callbacks — so the bindings_ bind race and the
  // timer/sub-teardown-vs-heartbeat race cannot occur. (SurveyPath's m_param_mutex
  // is retained defensively; see SurveyPath.h.)
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node->get_node_base_interface());

  executor->spin();
  rclcpp::shutdown();
  return 0;
}

