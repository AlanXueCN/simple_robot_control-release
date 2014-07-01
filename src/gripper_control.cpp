/*
 * gripper_control.cpp
 *
 *  Created on: Jul 27, 2010
 *      Author: christian
 */

#include <simple_robot_control/gripper_control.h>
 
namespace simple_robot_control {

  Gripper::Gripper(char gripper_side): 
    gripperside_str (&gripper_side, 1),
    gripper_client_("/" + gripperside_str + "_gripper_controller/gripper_action", true) {
    
    if (gripper_side !='l' && gripper_side != 'r') {
      ROS_ERROR("Gripper:specify l or r for arm side");
    }
    //wait for the gripper action server to come up
    while(!gripper_client_.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the %s_gripper_controller/gripper_action action server to come up", gripperside_str.c_str());
    }
  }

  Gripper::~Gripper() {
  }

  //Open the gripper
  bool Gripper::open(double effort) {
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = effort;  // Do not limit effort (negative)

    gripper_client_.sendGoal(open);
    gripper_client_.waitForResult(ros::Duration(5.0));
    if(gripper_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    }
    else {
      return false;
    }
  }

  //Close the gripper
  bool Gripper::close(double effort) {
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = -100.0;
    squeeze.command.max_effort = effort;  // Close gently

    gripper_client_.sendGoal(squeeze);
    gripper_client_.waitForResult(ros::Duration(5.0));
    if(gripper_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    }
    else {
      return false;
    }
  }

  bool Gripper::gripperToPos(double pos, double effort) {
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = pos;
    squeeze.command.max_effort = effort;  // Close gently

    gripper_client_.sendGoal(squeeze);
    gripper_client_.waitForResult(ros::Duration(1.5));
    if(gripper_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    }
    else {
      return false;
    }
  }

  bool Gripper::getOpeningWidth(double& opening_width) {
  	simple_robot_control::ReturnJointStates req;
  	req.request.name.push_back(gripperside_str + "_gripper_joint");

  	if(!ros::service::waitForService("return_joint_states", ros::Duration(5.0))) {
      ROS_ERROR("Gripper:Could not contact return_joint_states service. Did you run joint_state_listner?");
      return false;
  	}
  	if (!ros::service::call("return_joint_states",req)) {
      ROS_ERROR("Gripper service call failed");
  	}
  	opening_width =  req.response.position[0];
  	return true;
  }
}
