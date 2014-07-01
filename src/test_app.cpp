/*
 * test_app.cpp
 *
 *  Created on: Feb 03, 2011
 *      Author: Christian Bersch
 */

#include <ros/ros.h>
#include <simple_robot_control/robot_control.h>

void test_head(simple_robot_control::Robot robot) {
}

void test_torso(simple_robot_control::Robot robot) {
}


int main(int argc, char** argv){

	ros::init(argc, argv, "robot_control_test_app");
	ros::NodeHandle nh;

	//Create robot controller interface
	simple_robot_control::Robot robot;


    ROS_INFO("Testing the head movements!");
    robot.head.lookat("base_link", 2.0, -2.0, 1.0);
    ros::Duration(2.0).sleep();
    robot.head.lookat("base_link", tf::Vector3(2.0, 0.0, 1.0));
    ros::Duration(2.0).sleep();
    robot.head.lookat("l_gripper_tool_frame");

    //  TODO: these aren't tested yet:
    //bool lookat( const  geometry_msgs::PoseStampedConstPtr pose, double speed = DEFAULT_SPEED, string pointing_frame = DEFAULT_HEAD_POINTING_FRAME);
    //bool lookat( const tf::StampedTransform  pose, double speed = DEFAULT_SPEED, string pointing_frame = DEFAULT_HEAD_POINTING_FRAME);
    
    ROS_INFO("Testing the torso - should go from 0.15 to 0.3");
    robot.torso.move(0.15);
    ros::Duration(3.0).sleep();
    robot.torso.move(0.3);

    ROS_INFO("Testing the grippers - left should wind up open, right half-open");
    robot.left_gripper.close();
    robot.right_gripper.close();
    robot.left_gripper.open();
    robot.right_gripper.gripperToPos(0.04);
    double l_width, r_width;
    robot.left_gripper.getOpeningWidth(l_width);
    robot.right_gripper.getOpeningWidth(r_width);
    ROS_INFO("... final reported positions. Left: %0.2f, Right: %0.2f", l_width, r_width);

    ROS_INFO("Testing the base - robot should drive in a small square, then turn in place");
    robot.base.driveForward(0.1);
    robot.base.driveLeft(0.1, 0.1);
    robot.base.driveBack(0.1, 0.1);
    robot.base.driveRight(0.1, 0.1);
    ros::Duration(3.0).sleep();
    robot.base.turn(true, 0.4);
    ros::Duration(3.0).sleep();
    robot.base.turn(false, 0.4);
    ros::Duration(3.0).sleep();

    // TODO: not yet tested:
	//bool drive(double distance, const geometry_msgs::Twist& velocity);

	//do stuff with arms
    ROS_INFO("Now, for the arms!");
    ROS_INFO("Tucking/stretching...");
    robot.left_gripper.close();
    robot.right_gripper.close();
	robot.left_arm.tuck();
    robot.right_arm.tuck();
	robot.right_arm.stretch();
    robot.left_arm.stretch();

    ROS_INFO("rotating wrists ...");
    robot.left_arm.rotateWrist(-0.8);
    robot.right_arm.rotateWrist(0.8);

    ROS_INFO("Should be moving right arm to 2 traj points");
	double tuck_pos_right[] = { -0.4,0.0,0.0,-2.25,0.0,0.0,0.0, -0.01,1.35,-1.92,-1.68, 1.35,-0.18,0.31};
	std::vector<double> tuck_pos_vec(tuck_pos_right, tuck_pos_right + sizeof(tuck_pos_right)/sizeof(double));
	robot.right_arm.goToJointPos(tuck_pos_vec, 20.0, true);

	robot.right_arm.moveGripperToPosition(tf::Vector3(0.6,-0.1, 0.0), "torso_lift_link", simple_robot_control::Arm::FROM_ABOVE);
	robot.right_arm.moveGripperToPosition(tf::Vector3(0.8,-0.1, 0.1), "torso_lift_link", simple_robot_control::Arm::FRONTAL);

	tf::StampedTransform tf_l (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.8,0.1,0.0)), ros::Time::now(), "torso_lift_link","doesnt_matter");
	robot.left_arm.moveGripperToPose(tf_l);

    // this moves the L arm into the path of the r arm, causing planning to fail
    // it should succeed after the l arm moves out of the way
    ROS_INFO("Testing collision avoidance. ");
	robot.right_arm.stretch();
    robot.left_arm.stretch();
    robot.left_arm.moveGripperToPosition(tf::Vector3(0.65, -0.25, 0.65), "base_link");

	double tuck_pos_step1[] = { -0.4,0.0,0.0,-2.25,0.0,0.0,0.0};
	std::vector<double> tuck_pos_vec1(tuck_pos_step1, tuck_pos_step1 + sizeof(tuck_pos_step1)/sizeof(double));
	robot.right_arm.goToJointPosWithCollisionChecking(tuck_pos_vec1, 20.0, true);

	double tuck_pos_step2[] = {0.33, 0.84, -1.5, -1.5, 2.8, -1.6, -2.6};
	std::vector<double> tuck_pos_vec2(tuck_pos_step2, tuck_pos_step2 + sizeof(tuck_pos_step2)/sizeof(double));
    ROS_INFO("...take 1");
	robot.right_arm.goToJointPosWithCollisionChecking(tuck_pos_vec2, 20.0, true);
    robot.left_arm.stretch();
    ROS_INFO("...take 2");
    robot.right_arm.goToJointPosWithCollisionChecking(tuck_pos_vec2, 20.0, true);

    // Now, testing the position with collision checking
    ROS_INFO("And again - Right arm should fail until left moves");
	robot.right_arm.moveGripperToPositionWithCollisionChecking(tf::Vector3(0.6,-0.1, 0.0), "torso_lift_link", simple_robot_control::Arm::FROM_ABOVE);
	robot.right_arm.moveGripperToPositionWithCollisionChecking(tf::Vector3(0.8,-0.1, 0.1), "torso_lift_link", simple_robot_control::Arm::FRONTAL);

	robot.left_arm.moveGripperToPoseWithCollisionChecking(tf_l);
    ROS_INFO("...take 1");
	robot.right_arm.moveGripperToPoseWithCollisionChecking(tf_l);
    robot.left_arm.stretch();
    ROS_INFO("...take 2");
	robot.right_arm.moveGripperToPoseWithCollisionChecking(tf_l);

    ROS_INFO("Finally, testing orientation constraints. Third motion should keep gripper level");
	tf::StampedTransform tf_1 (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.8,0.1,0.0)), ros::Time::now(), "torso_lift_link","doesnt_matter");
	tf::StampedTransform tf_2 (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.2,-0.7,0.0)), ros::Time::now(), "torso_lift_link","doesnt_matter");

	robot.right_arm.moveGripperToPoseWithCollisionChecking(tf_1);
    ros::Duration(3.0).sleep();
	robot.right_arm.moveGripperToPoseWithCollisionChecking(tf_2);
    ros::Duration(3.0).sleep();
	robot.right_arm.moveGripperToPoseWithOrientationConstraints(tf_1, true, false, false);
    ros::Duration(3.0).sleep();
	robot.right_arm.moveGripperToPoseWithCollisionChecking(tf_2);

    return 0;
}
