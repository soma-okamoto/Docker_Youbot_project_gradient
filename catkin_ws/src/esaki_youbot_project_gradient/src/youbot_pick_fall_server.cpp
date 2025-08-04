#include <server/youbot_pick_and_place_Callback.h>
#include <server/Move_arm_1_server.h>
#include <server/Move_arm_2_server.h>
#include <server/Search_obj_server.h>
#include <server/Gripper_server.h>
#include <server/Alignment_pick_fall_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_pick_fall_server");

  // Youbot_Pick_and_Place_Server Youbot_Pick_and_Place_Server();

  Youbot_Pick_and_Place_Callback Youbot_Pick_and_Place_Callback;

  Move_Arm_1_Server           Move_Arm_1_Server("Move_arm_1");
  Move_Arm_2_Server           Move_Arm_2_Server("Move_arm_2");
  Search_Obj_Server           Search_Obj_Server("Search_obj");
  Alignment_Pick_Fall_Server  Alignment_Pick_Fall_Server("Alignment_pick_fall");
  Gripper_Server              Gripper_Server("Gripper");

  ros::spin();

  ros::Rate loop_rate(50);

  ROS_INFO("main end");

  return 0;
}
