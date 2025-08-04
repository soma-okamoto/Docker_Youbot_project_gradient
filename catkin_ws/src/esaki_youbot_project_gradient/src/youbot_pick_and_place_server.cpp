#include <server/youbot_pick_and_place_Callback.h>
#include <server/Search_obj_server.h>
#include <server/Approach_obj_server.h>
#include <server/Move_arm_1_server.h>
#include <server/Move_arm_2_server.h>
#include <server/Turn_arm_1_server.h>
#include <server/Turn_arm_2_server.h>
#include <server/Gripper_server.h>
#include <server/Search_place_server.h>
#include <server/Approach_place_server.h>
#include <server/Alignment_server.h>
#include <server/Approach_smartglass_server.h>
#include <server/Turn_smartglass_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_pick_and_place_server");

  // Youbot_Pick_and_Place_Server Youbot_Pick_and_Place_Server();

  Youbot_Pick_and_Place_Callback Youbot_Pick_and_Place_Callback;

  Search_Obj_Server           Search_Obj_Server("Search_obj");
  Approach_Obj_Server         Approach_Obj_Server("Approach_obj");
  Move_Arm_1_Server           Move_Arm_1_Server("Move_arm_1");
  Move_Arm_2_Server           Move_Arm_2_Server("Move_arm_2");
  Turn_Arm_1_Server           Turn_Arm_1_Server("Turn_arm_1");
  Turn_Arm_2_Server           Turn_Arm_2_Server("Turn_arm_2");
  Alignment_Server            Alignment_Server("Alignment");
  Gripper_Server              Gripper_Server("Gripper");
  Search_Place_Server         Search_Place_Server("Search_place");
  Approach_Place_Server       Approach_Place_Server("Approach_place");
  Approach_Smartglass_Server  Approach_Smartglass_Server("Approach_smartglass");

  ros::spin();

  ros::Rate loop_rate(50);

  ROS_INFO("main end");

  return 0;
}
