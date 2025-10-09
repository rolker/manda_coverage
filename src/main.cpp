/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: MIT                                             */
/*    FILE: main.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include "SurveyPath.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SurveyPath>();

  rclcpp::spin(node->get_node_base_interface());  
  rclcpp::shutdown();
  return 0;
}

