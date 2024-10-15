/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: Launches heading_node and runs the initial calibration.
# Contacts: soyoung.kim@antobot.ai
# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <antobot_heading/heading.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "heading_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  heading::heading auto_calibration_node(nh);
  auto_calibration_node.initialCalibration();

  ros::spin();

  return(0);
}