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
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node->get_node_base_interface());

  executor->spin();
  rclcpp::shutdown();
  return 0;
}

