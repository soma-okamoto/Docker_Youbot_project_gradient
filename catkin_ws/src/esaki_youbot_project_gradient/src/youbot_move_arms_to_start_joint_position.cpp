
#include <client/Move_arm_1_client.h>
#include <client/Move_arm_2_client.h>
#include <DegToRad.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_move_arms_to_start_joint_position");
  ros::NodeHandle nh;

  std::vector<double> arm_1_joint_command, arm_2_joint_command;
  arm_1_joint_command.resize(5);
  arm_2_joint_command.resize(5);

  nh.getParam("arm1_joint1", arm_1_joint_command[0]);
  nh.getParam("arm1_joint2", arm_1_joint_command[1]);
  nh.getParam("arm1_joint3", arm_1_joint_command[2]);
  nh.getParam("arm1_joint4", arm_1_joint_command[3]);
  nh.getParam("arm1_joint5", arm_1_joint_command[4]);

  nh.getParam("arm2_joint1", arm_2_joint_command[0]);
  nh.getParam("arm2_joint2", arm_2_joint_command[1]);
  nh.getParam("arm2_joint3", arm_2_joint_command[2]);
  nh.getParam("arm2_joint4", arm_2_joint_command[3]);
  nh.getParam("arm2_joint5", arm_2_joint_command[4]);

  // nh.getParam("base_x", base_x);
  // nh.getParam("base_y", base_y);
  // nh.getParam("base_z", base_z);


  Move_Arm_1_Client     move_arm_1;
  Move_Arm_2_Client     move_arm_2;

  move_arm_1.Move_arm_1(DegToRad(arm_1_joint_command[0]), DegToRad(arm_1_joint_command[1]), DegToRad(arm_1_joint_command[2]), DegToRad(arm_1_joint_command[3]), DegToRad(arm_1_joint_command[4]));
  move_arm_2.Move_arm_2(DegToRad(arm_2_joint_command[0]), DegToRad(arm_2_joint_command[1]), DegToRad(arm_2_joint_command[2]), DegToRad(arm_2_joint_command[3]), DegToRad(arm_2_joint_command[4]));
  

  return 0;
}