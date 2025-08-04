#include <client/youbot_pick_and_place_client.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_pick_fall_client");
  ros::NodeHandle nh;

  Search_Obj_Client           search_obj;
  Alignment_Pick_Fall_Client  alignment_pick_fall;
  Move_Arm_1_Client           move_arm_1;
  Move_Arm_2_Client           move_arm_2;
  Gripper_Client              gripper;

  // std::cout << "===========================================" << std::endl;

  move_arm_1.Move_arm_1(DegToRad(0), DegToRad(30), DegToRad(30), DegToRad(30), DegToRad(0));
  move_arm_2.Move_arm_2(DegToRad(0), DegToRad(-40), DegToRad(80), DegToRad(80), DegToRad(0));

  // gripper.Gripper("open");

  search_obj.Search_obj(&search_obj_success, &obj_label, &base_r_founded_obj);
  move_arm_2.Move_arm_2(base_r_founded_obj, DegToRad(-40), DegToRad(80), DegToRad(80), DegToRad(0));
  search_obj.Search_obj(&search_obj_success, &obj_label, &base_r_founded_obj);

  move_arm_1.Move_arm_1(base_r_approached_obj, DegToRad(0), DegToRad(110), DegToRad(-20), DegToRad(0));

  alignment_pick_fall.Alignment_pick_fall(base_r_approached_obj, &alignment_pick_success);

  gripper.Gripper("grasp", &grasp_success);
  std::cout << "grasp_success = " << grasp_success << std::endl;

  move_arm_1.Move_arm_1(base_r_approached_obj, DegToRad(0), DegToRad(45), DegToRad(45), DegToRad(0));

  ros::spin();

  ros::Rate loop_rate(50);

  return 0;
}
