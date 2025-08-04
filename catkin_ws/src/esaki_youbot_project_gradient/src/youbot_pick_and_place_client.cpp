#include <client/youbot_pick_and_place_client.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_pick_and_place_client");
  ros::NodeHandle nh;

  Search_Obj_Client           search_obj;
  Approach_Obj_Client         approach_obj;
  Move_Arm_1_Client           move_arm_1;
  Move_Arm_2_Client           move_arm_2;
  Turn_Arm_1_Client           turn_arm_1;
  Turn_Arm_2_Client           turn_arm_2;
  Gripper_Client              gripper;
  Search_Place_Client         search_place;
  Approach_Place_Client       approach_place;
  Alignment_Client            alignment;

  founded_place_1.from_odom.pose.position.x = 100.0;
  founded_place_1.from_odom.pose.position.y = 0.0;
  founded_place_1.from_odom.pose.position.z = 0.0;
  founded_place_1.from_base.pose.position.x = 100.0;
  founded_place_1.from_base.pose.position.y = 0.0;
  founded_place_1.from_base.pose.position.z = 0.0;

  founded_place_2.from_odom.pose.position.x = 100.0;
  founded_place_2.from_odom.pose.position.y = 0.0;
  founded_place_2.from_odom.pose.position.z = 0.0;
  founded_place_2.from_base.pose.position.x = 100.0;
  founded_place_2.from_base.pose.position.y = 0.0;
  founded_place_2.from_base.pose.position.z = 0.0;

  move_arm_1.Move_arm_1(DegToRad(0), DegToRad(30), DegToRad(30), DegToRad(30), DegToRad(0));
  move_arm_2.Move_arm_2(DegToRad(0), DegToRad(-40), DegToRad(70), DegToRad(80), DegToRad(0));

  while((count_1<place.target.size() && count_2<place.target.size()) && ros::ok()){

    gripper.Gripper("close");

    grasp_success = false;
    approach_place_success = false;

    move_arm_1.Move_arm_1(DegToRad(0), DegToRad(30), DegToRad(30), DegToRad(30), DegToRad(0));
    move_arm_2.Move_arm_2(DegToRad(-40), DegToRad(70), DegToRad(80), DegToRad(0));

    while(!approach_place_success && ros::ok()){

      move_arm_1.Move_arm_1(DegToRad(0), DegToRad(30), DegToRad(30), DegToRad(30), DegToRad(0));
      move_arm_2.Move_arm_2(DegToRad(-40), DegToRad(70), DegToRad(80), DegToRad(0));

      approach_obj_success = false;

      while(!approach_obj_success && ros::ok()){

        std::cout << "founded_place_1.from_base.pose.position = \n" << founded_place_1.from_base.pose.position << std::endl;
        std::cout << "founded_place_1.from_odom.pose.position = \n" << founded_place_1.from_odom.pose.position << std::endl;
        std::cout << "founded_place_2.from_base.pose.position = \n" << founded_place_2.from_base.pose.position << std::endl;
        std::cout << "founded_place_2.from_odom.pose.position = \n" << founded_place_2.from_odom.pose.position << std::endl;

        search_obj.Search_obj(founded_place_1, founded_place_2, &search_obj_success, &founded_obj);
        // std::cout << "search_obj_success = " << search_obj_success << std::endl;
        std::cout << "founded_obj.label = " << founded_obj.label << std::endl;
        std::cout << "founded_obj.from_base.pose.position = \n" << founded_obj.from_base.pose.position << std::endl;
        std::cout << "founded_obj.from_odom.pose.position = \n" << founded_obj.from_odom.pose.position << std::endl;

        // move_arm_2.Move_arm_2(founded_obj.from_base.pose, DegToRad(-30), DegToRad(60), DegToRad(80), DegToRad(0));
        turn_arm_2.Turn_arm_2(founded_obj.from_odom.pose, DegToRad(-40), DegToRad(70), DegToRad(80), DegToRad(0));

        search_obj.Search_obj(founded_place_1, founded_place_2, &search_obj_success, &founded_obj);
        // // std::cout << "search_obj_success = " << search_obj_success << std::endl;
        std::cout << "founded_obj.label = " << founded_obj.label << std::endl;
        std::cout << "founded_obj.from_base.pose.position = \n" << founded_obj.from_base.pose.position << std::endl;
        std::cout << "founded_obj.from_odom.pose.position = \n" << founded_obj.from_odom.pose.position << std::endl;

        approach_obj.Approach_obj(founded_obj, &approach_obj_success, &approached_obj, &theta_a, &theta_5);
        std::cout << "approach_obj_success = " << approach_obj_success << std::endl;
        // std::cout << "approached_obj.label = " << approached_obj.label << std::endl;
        std::cout << "approached_obj.from_base.pose.position = \n" << approached_obj.from_base.pose.position << std::endl;
        std::cout << "approached_obj.from_odom.pose.position = \n" << approached_obj.from_odom.pose.position << std::endl;
        std::cout << "approached_obj.stand = " << approached_obj.stand << std::endl;
        std::cout << "theta_5 = " << RadToDeg(theta_5) << std::endl;
      }

      gripper.Gripper("open");

      if(approached_obj.stand)
      {
      turn_arm_1.Turn_arm_1(approached_obj.from_odom.pose, DegToRad(0), DegToRad(110), DegToRad(-20), DegToRad(0));
      alignment.Alignment(approached_obj, DegToRad(90), DegToRad(0), &alignment_pick_success);
      }
      else
      {
      turn_arm_1.Turn_arm_1(approached_obj.from_odom.pose, DegToRad(20), DegToRad(40), DegToRad(60), theta_5);
      alignment.Alignment(approached_obj, DegToRad(180), theta_5, &alignment_pick_success);
      }

      // std::cout << "alignment_pick_success = " << alignment_pick_success << std::endl;

      gripper.Gripper("grasp", &grasp_success);
      std::cout << "grasp_success = " << grasp_success << std::endl;

      move_arm_1.Move_arm_1(DegToRad(0), DegToRad(45), DegToRad(45), DegToRad(0));

      // approach_place_success = false;

      // founded_obj.label = "bottle";

      while(grasp_success && !approach_place_success && ros::ok()){
        std::cout << "founded_obj.label = " << founded_obj.label << std::endl;

        if(founded_obj.label=="bottle"){
          std::cout << "founded_place_1.from_base.pose.position = \n" << founded_place_1.from_base.pose.position << std::endl;
          std::cout << "founded_place_1.from_odom.pose.position = \n" << founded_place_1.from_odom.pose.position << std::endl;
          // move_arm_2.Move_arm_2(founded_place_1.from_base.pose, DegToRad(-45), DegToRad(75), DegToRad(75), DegToRad(0));
          turn_arm_2.Turn_arm_2(founded_place_1.from_odom.pose, DegToRad(-45), DegToRad(75), DegToRad(75), DegToRad(0));

          search_place.Search_place(founded_obj.label, founded_place_1, founded_place_2, &grasp_success, &search_place_success, &founded_place_1, &founded_place_2);
          // std::cout << "search_place_success = " << search_place_success << std::endl;
          // std::cout << "founded_place_1.label = \n" << founded_place_1.label << std::endl;
          std::cout << "founded_place_1.from_base.pose.position = \n" << founded_place_1.from_base.pose.position << std::endl;
          std::cout << "founded_place_1.from_odom.pose.position = \n" << founded_place_1.from_odom.pose.position << std::endl;

          std::cout << "count_1 = " << count_1 << std::endl;
          approach_place.Approach_place(founded_obj.label, founded_place_1, count_1, &grasp_success, &approach_place_success, &approached_place);
          std::cout << "grasp_success = " << grasp_success << std::endl;
          std::cout << "approach_place_success = " << approach_place_success << std::endl;
          // std::cout << "approached_place.label = \n" << approached_place.label << std::endl;
          std::cout << "approached_place.from_base.pose.position = \n" << approached_place.from_base.pose.position << std::endl;
          std::cout << "approached_place.from_odom.pose.position = \n" << approached_place.from_odom.pose.position << std::endl;

          if(approach_place_success){
            count_1++;
          }
        }
        else if(founded_obj.label=="cup"){
          std::cout << "founded_place_2.from_base.pose.position = \n" << founded_place_2.from_base.pose.position << std::endl;
          std::cout << "founded_place_2.from_odom.pose.position = \n" << founded_place_2.from_odom.pose.position << std::endl;
          // move_arm_2.Move_arm_2(founded_place_2.from_base.pose, DegToRad(-45), DegToRad(75), DegToRad(75), DegToRad(0));
          turn_arm_2.Turn_arm_2(founded_place_2.from_odom.pose, DegToRad(-45), DegToRad(75), DegToRad(75), DegToRad(0));

          search_place.Search_place(founded_obj.label, founded_place_1, founded_place_2, &grasp_success, &search_place_success, &founded_place_1, &founded_place_2);
          // std::cout << "search_place_success = " << search_place_success << std::endl;
          // std::cout << "founded_place_2.label = \n" << founded_place_2.label << std::endl;
          std::cout << "founded_place_2.from_base.pose.position = \n" << founded_place_2.from_base.pose.position << std::endl;
          std::cout << "founded_place_2.from_odom.pose.position = \n" << founded_place_2.from_odom.pose.position << std::endl;

          std::cout << "count_2 = " << count_2 << std::endl;
          approach_place.Approach_place(founded_obj.label, founded_place_2, count_2, &grasp_success, &approach_place_success, &approached_place);
          std::cout << "grasp_success = " << grasp_success << std::endl;
          std::cout << "approach_place_success = " << approach_place_success << std::endl;
          // std::cout << "approached_place.label = \n" << approached_place.label << std::endl;
          std::cout << "approached_place.from_base.pose.position = \n" << approached_place.from_base.pose.position << std::endl;
          std::cout << "approached_place.from_odom.pose.position = \n" << approached_place.from_odom.pose.position << std::endl;

          if(approach_place_success){
            count_2++;
          }
        }
      }
    }

    turn_arm_1.Turn_arm_1(approached_place.from_odom.pose, DegToRad(0), DegToRad(110), DegToRad(-20), DegToRad(0));

    alignment.Alignment(approached_place, DegToRad(90), DegToRad(0), &alignment_place_success);
    std::cout << "alignment_place_success = " << alignment_place_success << std::endl;

    gripper.Gripper("open");

    move_arm_1.Move_arm_1(DegToRad(30), DegToRad(30), DegToRad(30), DegToRad(0));
  }

  move_arm_1.Move_arm_1(DegToRad(0), DegToRad(30), DegToRad(30), DegToRad(30), DegToRad(0));
  move_arm_2.Move_arm_2(DegToRad(0), DegToRad(-40), DegToRad(70), DegToRad(80), DegToRad(0));

  gripper.Gripper("close");

  std::cout << "finished " << std::endl;

  ros::spin();

  ros::Rate loop_rate(50);

  return 0;
}
