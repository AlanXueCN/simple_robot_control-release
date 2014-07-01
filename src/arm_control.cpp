/*
 * arm_control.cpp
 *
 *  Created on: Jul 27, 2010
 *      Author: christian
 */

#define HALF_SQRT_TWO 0.707106781

#include <simple_robot_control/arm_control.h>

namespace simple_robot_control{

  Arm::Arm(char arm_side, bool collision_checking): 
    armside_str (&arm_side, 1), 
    move_arm_client_(NULL),
    traj_client_(string(armside_str) + "_arm_controller/joint_trajectory_action", true)
  {
    if (arm_side !='l' && arm_side != 'r' ) {
      ROS_ERROR("Arm:specify l or r for arm side");
    }
    // First, the joint names, which apply to all waypoints
    joint_names.reserve(7);
    joint_names.push_back(armside_str+"_shoulder_pan_joint");
    joint_names.push_back(armside_str +"_shoulder_lift_joint");
    joint_names.push_back(armside_str +"_upper_arm_roll_joint");
    joint_names.push_back(armside_str +"_elbow_flex_joint");
    joint_names.push_back(armside_str +"_forearm_roll_joint");
    joint_names.push_back(armside_str +"_wrist_flex_joint");
    joint_names.push_back(armside_str +"_wrist_roll_joint");

    std::string ik_service_name;
    ik_service_name = "compute_ik";

    if (!ros::service::waitForService(ik_service_name, ros::Duration(5.0))) {
      ROS_ERROR("Could not find ik server %s", ik_service_name.c_str());
    }
    ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>(ik_service_name);

    // wait for action server to come up
    if(!traj_client_.waitForServer(ros::Duration(15.0))) {
      ROS_ERROR("Could not find joint_trajectory_action server %s", 
                (armside_str + "_arm_controller/joint_trajectory_action").c_str());
    }

    if(arm_side == 'r') {
      group_name = "right_arm";
    } else if(arm_side == 'l') {
      group_name = "left_arm";
    }
  
    if (collision_checking) {
      move_arm_client_ = new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>("move_group", true);
      if(!move_arm_client_->waitForServer(ros::Duration(5.0))) {
        ROS_INFO("could not find collision move arm client");
        move_arm_client_ = NULL;
      }
    }
    updateJointStatePos();
  }

  //! Clean up the action client
  Arm::~Arm()
  {
    delete move_arm_client_;
  }

  bool Arm::moveWristRollLinkToPose(const tf::StampedTransform& tf,  double max_time, 
                                    bool wait, vector<double>* ik_seed_pos) {
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);

    tf::poseStampedTFToMsg(tf_pose,pose);
    return moveWristRollLinkToPose(pose, max_time, wait, ik_seed_pos);
  }

  bool Arm::moveWristRollLinkToPoseWithCollisionChecking(const tf::StampedTransform& tf,  double max_time, 
                                                         bool wait, std::string planner) {
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);

    tf::poseStampedTFToMsg(tf_pose,pose);
    return moveWristRollLinkToPoseWithCollisionChecking(pose, max_time, wait, planner);
  }

  bool Arm::goToJointPosWithCollisionChecking(const vector<double>& positions, double max_time, bool wait) {
    if (!move_arm_client_) {
      ROS_ERROR("collosion checking arm server has not been started");
      return false;
    }
    moveit_msgs::MoveGroupGoal goal;

    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    goal.request.group_name = group_name;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = max_time;

    // TODO: specify which planner?
    goal.request.planner_id = std::string("");

    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0].joint_constraints.resize(joint_names.size());

    if(positions.size() != joint_names.size()) {
      ROS_ERROR("called goToJointPosWithCollisionChecking with %lu positions, but there are only %lu joints", 
                positions.size(), joint_names.size());
      return false;
    }

    for (unsigned int idx = 0 ; idx < joint_names.size(); idx++) {
      goal.request.goal_constraints[0].joint_constraints[idx].joint_name = joint_names[idx];
      goal.request.goal_constraints[0].joint_constraints[idx].position = positions[idx];
      goal.request.goal_constraints[0].joint_constraints[idx].tolerance_below = 0.0001;
      goal.request.goal_constraints[0].joint_constraints[idx].tolerance_above = 0.0001;
      goal.request.goal_constraints[0].joint_constraints[idx].weight = 1.0;
    }

    bool finished_within_time = false;
    bool success = false;
    move_arm_client_->sendGoal(goal);
    finished_within_time = move_arm_client_->waitForResult(ros::Duration(5*max_time));
    if (!finished_within_time) {
      move_arm_client_->cancelGoal();
      ROS_INFO("Timed out achieving  JointPos goal");
    } else {
      actionlib::SimpleClientGoalState state = move_arm_client_->getState();
      success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    }
    return finished_within_time && success;
  }

  bool Arm::moveWristRollLinkToPoseWithCollisionChecking(const geometry_msgs::PoseStamped& pose, 
                                                         double max_time, bool wait, std::string planner) {
    if (!move_arm_client_) {
      ROS_ERROR("collision checking arm server has not been started");
      return false;
    }

    moveit_msgs::MoveGroupGoal goal;

    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    goal.request.group_name = group_name;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = max_time;

    // TODO: specify which planner?
    goal.request.planner_id = std::string("");
    /**
       if (planner == "chomp") {
       ROS_INFO("using chomp planner");
       goalA.planner_service_name = std::string("/chomp_planner_longrange/plan_path");
       }else{
       ROS_INFO("using ompl planner");
       goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
       }
    **/

    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0].position_constraints.resize(1);
    goal.request.goal_constraints[0].orientation_constraints.resize(1);

    goal.request.goal_constraints[0].position_constraints[0].header.frame_id = pose.header.frame_id;
    goal.request.goal_constraints[0].position_constraints[0].link_name = armside_str + "_wrist_roll_link";
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives.resize(1);
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].type = 2; //SolidPrimitive.SPHERE
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.push_back(0.0001);
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.resize(1);
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position = pose.pose.position;

    goal.request.goal_constraints[0].orientation_constraints[0].header.frame_id = pose.header.frame_id;
    goal.request.goal_constraints[0].orientation_constraints[0].orientation = pose.pose.orientation;
    goal.request.goal_constraints[0].orientation_constraints[0].link_name = armside_str + "_wrist_roll_link";
    goal.request.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance = 0.001;
    goal.request.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance = 0.001;
    goal.request.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance = 0.001;
    goal.request.goal_constraints[0].orientation_constraints[0].weight = 1.0;

    move_arm_client_->sendGoal(goal);
    bool finished_before_timeout = false;
    bool success = false;

    if (wait) {
      finished_before_timeout = move_arm_client_->waitForResult(ros::Duration(5*max_time));
      if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = move_arm_client_->getState();
        success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      }
    }
    return finished_before_timeout && success;
  }

  vector<double> Arm::getCurrentJointAngles() {
    updateJointStatePos();
    return current_joint_angles_;
  }

  bool Arm::getIK(const geometry_msgs::PoseStamped& pose,  vector<double>& joint_angles, vector<double>* ik_seed_pos) {

    moveit_msgs::GetPositionIK service_call;

    service_call.request.ik_request.group_name = group_name;
    service_call.request.ik_request.pose_stamped = pose;

    service_call.request.ik_request.robot_state.joint_state.name = joint_names;
    if (ik_seed_pos) {
      service_call.request.ik_request.robot_state.joint_state.position = *ik_seed_pos;
    } else {
      service_call.request.ik_request.robot_state.joint_state.position = getCurrentJointAngles();
    }

    if (!ik_service_client_.call(service_call)) {
      ROS_ERROR("ik_service_call failed");
      return false;
    }

    if (service_call.response.error_code.val != 1) {
      ROS_ERROR("Could not get valid IK: error code %d", service_call.response.error_code.val);
      return false;
    }

    // The ik response includes ALL the joints for our robot; we only want to keep the ones
    // corresponding to this arm
    joint_angles.clear();
    for (int ii=0; ii < joint_names.size(); ii++) {
      for (int jj=0; jj<service_call.response.solution.joint_state.name.size(); jj++) {
        std::string n1 = joint_names[ii];
        std::string n2 = service_call.response.solution.joint_state.name[jj];
        double pos = service_call.response.solution.joint_state.position[jj];
        if (0 == n1.compare(n2)) {
          //ROS_INFO("for joint %s, indices are %d and %d", n1.c_str(), ii, jj);
          joint_angles.push_back(pos);
        }
      }
    }
    return true;
  }


  bool Arm::moveWristRollLinkToPose(const geometry_msgs::PoseStamped& pose, double max_time, 
                                    bool wait, vector<double>* ik_seed_pos) {
    vector<double> joint_angles;
    if (!getIK(pose ,joint_angles , ik_seed_pos)) {
      return false;
    }
    return goToJointPos(joint_angles, max_time, wait );
  }

  bool Arm::goToJointPos (const double* positions , int num_positions, double max_time , bool wait) {
    std::vector<double> pos_vec (positions, positions + 7 * num_positions);
    return goToJointPos(pos_vec, max_time, wait);
  }

  bool Arm::goToJointPos (const vector<double>& positions , double max_time, bool wait) {
    if (positions.size() % 7 != 0) {
      ROS_ERROR("you must specify 7 (or a multiple thereof) for the arm joint positions");
    }
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.joint_names = joint_names;
    unsigned int num_waypoints = positions.size() / 7;
    goal.trajectory.points.resize(num_waypoints);
    vector<double>::const_iterator it = positions.begin();
    vector<double>::const_iterator it_end = positions.begin() + 7;

    for (unsigned int i=0; i <num_waypoints; i++) {
      goal.trajectory.points[i].positions.insert(goal.trajectory.points[i].positions.begin(), it, it_end);
      goal.trajectory.points[i].velocities.resize(7);
      goal.trajectory.points[i].time_from_start = ros::Duration( (i+1) * max_time / num_waypoints);
      it =it_end;
      it_end+=7;
    }

    goal.trajectory.header.stamp = ros::Time::now();
    traj_client_.sendGoal(goal);
    bool finished_before_timeout = false;
    if (wait) {
      finished_before_timeout = traj_client_.waitForResult(ros::Duration(3*max_time));
      if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = traj_client_.getState();
      }
    }
    return finished_before_timeout;
  }

  bool Arm::updateJointStatePos() {
    simple_robot_control::ReturnJointStates req;
    req.request.name = joint_names;

    if(!ros::service::waitForService("return_joint_states", ros::Duration(5.0))) {
      ROS_ERROR("Arm:Could not contact return_joint_states service. Did you run joint_state_listner?");
      return false;
    }
    if (!ros::service::call("return_joint_states",req)) {
      ROS_ERROR("Arm service call failed");
    }
    current_joint_angles_=  req.response.position;
    return true;
  }

  bool Arm::rotateWrist(double radians, double wrist_speed, bool wait) {
    updateJointStatePos();
    vector<double> new_pos = current_joint_angles_;
    new_pos[6] += radians;
    return goToJointPos(new_pos, std::abs(radians) / 2 / 3.14 * wrist_speed, wait);
  }

  bool Arm::isAtPos(const std::vector<double>& pos_vec) {
    const double epsilon= 0.1;
    updateJointStatePos();
    int pos = pos_vec.size() - 7;
    for (int i=0; i<7; i++) {
      if (std::abs(current_joint_angles_[i] - pos_vec[pos + i]) > epsilon) {
        return false;
      }
    }
    return true;
  }

  bool Arm::tuck() {
    std::vector<double> tuck_pos_vec;
    if (armside_str == "r") {
      double tuck_pos[] = { -0.4,0.0,0.0,-2.25,0.0,0.0,0.0, -0.01,1.35,-1.92,-1.68, 1.35,-0.18,0.31};
      tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+14);
    } else {
      double tuck_pos[] = {   0.4,0.0,0.0,-2.25,0.0,0.0,0.0, -0.05,1.31,1.38,-2.06,1.69,-2.02,2.44};
      tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+14);
    }
    if (!isAtPos(tuck_pos_vec)) {
      return goToJointPos(tuck_pos_vec);
    }
    return true;
  }

  bool Arm::stretch() {
    std::vector<double> stretch_pos_vec;
    if (armside_str == "r") {
      double stretch_pos[] = {-1.634, -0.039, -0.324, -0.131, 31.779, 0.004, 24.986};
      stretch_pos_vec.insert(stretch_pos_vec.begin(),stretch_pos, stretch_pos+7);
    } else {
      double stretch_pos[] = {   1.613, -0.105, 0.336, -0.033, -6.747, 0.014, 0.295};
      stretch_pos_vec.insert(stretch_pos_vec.begin(),stretch_pos, stretch_pos+7);
    }
    if (!isAtPos(stretch_pos_vec)) {
      return goToJointPos(stretch_pos_vec);
    }
    return true;
  }

  tf::StampedTransform Arm::gripperToWrist(const tf::StampedTransform& pose, double gripper_length) {
    tf::Vector3 offset(gripper_length,0,0);
    tf::Transform rot(pose.getRotation());
    tf::StampedTransform out(tf::Transform(pose.getRotation(), pose.getOrigin() - rot * offset), 
                             pose.stamp_, pose.frame_id_, pose.child_frame_id_);
    return out;
  }

  tf::StampedTransform Arm::wristToGripper(const tf::StampedTransform& pose, double gripper_length) {
    tf::Vector3 offset(gripper_length,0,0);
    tf::Transform rot(pose.getRotation());
    tf::StampedTransform out(tf::Transform(pose.getRotation(), pose.getOrigin() + rot * offset), 
                             pose.stamp_, pose.frame_id_, pose.child_frame_id_);
    return out;
  }

  tf::StampedTransform Arm::makePose(const tf::Vector3& position, string frame_id, approach_direction_t approach) {
    tf::StampedTransform tf(tf::Transform(tf::Quaternion(0,0,0,1), position), ros::Time(), frame_id, "/doesntmatter");
    tf::TransformListener tf_listener;
    tf::StampedTransform tf_sourceframe_in_baselink;
    
    tf_listener.waitForTransform(  "/base_link", tf.frame_id_, ros::Time(0), ros::Duration(0.5));
    tf_listener.lookupTransform(  "/base_link", tf.frame_id_, ros::Time(), tf_sourceframe_in_baselink);
    tf::StampedTransform tf_pose_in_baselink (tf::Transform(tf_sourceframe_in_baselink * tf), 
                                              tf.stamp_, "/base_link", "/doesntmatter");

    switch (approach) {
    case (FRONTAL):
      tf_pose_in_baselink.setRotation(tf::Quaternion( 0, 0 , 0, 1)); break;
    case (FROM_BELOW):
      tf_pose_in_baselink.setRotation(tf::Quaternion( HALF_SQRT_TWO , 0, HALF_SQRT_TWO , 0)); break;
    case (FROM_RIGHT_SIDEWAYS):
      tf_pose_in_baselink.setRotation(tf::Quaternion( 0 , 0, HALF_SQRT_TWO , HALF_SQRT_TWO)); break;
    case (FROM_RIGHT_UPRIGHT):
      tf_pose_in_baselink.setRotation(tf::Quaternion( -0.5 , -0.5, 0.5 , 0.5)); break;
    case (FROM_ABOVE):
      tf_pose_in_baselink.setRotation(tf::Quaternion( HALF_SQRT_TWO , 0, -HALF_SQRT_TWO , 0)); break;
    case (FROM_LEFT_SIDEWAYS):
      tf_pose_in_baselink.setRotation(tf::Quaternion( -HALF_SQRT_TWO , HALF_SQRT_TWO , 0 , 0)); break;
    case (FROM_LEFT_UPRIGHT):
      tf_pose_in_baselink.setRotation(tf::Quaternion( -0.5 , 0.5, -0.5 , 0.5)); break;
    }
    return tf_pose_in_baselink;
  }

  bool Arm::moveGripperToPosition(const tf::Vector3& position, string frame_id, approach_direction_t approach, 
                                  double max_time , bool wait, vector<double>* ik_seed_pos) {
    tf::StampedTransform tf_pose_in_baselink_new(gripperToWrist(makePose(position,  frame_id,  approach)));
    return moveWristRollLinkToPose(tf_pose_in_baselink_new, max_time, wait, ik_seed_pos);
  }

  bool Arm::moveGripperToPose(const tf::StampedTransform& tf, double max_time, bool wait, vector<double>* ik_seed_pos) {
    tf::StampedTransform tf_new(gripperToWrist(tf));
    return moveWristRollLinkToPose(tf_new, max_time, wait, ik_seed_pos);
  }

  bool Arm::moveGripperToPoseWithCollisionChecking(const tf::StampedTransform& tf, double max_time, 
                                                   bool wait, std::string planner) {
    tf::StampedTransform tf_new(gripperToWrist(tf));
    return moveWristRollLinkToPoseWithCollisionChecking(tf_new, max_time, wait, planner);
  }

  bool Arm::moveGripperToPositionWithCollisionChecking(const tf::Vector3& position, string frame_id, 
                                                       approach_direction_t approach, double max_time, 
                                                       bool wait,std::string planner) {
    tf::StampedTransform tf_pose_in_baselink_new(gripperToWrist(makePose(position, frame_id, approach)));
    return moveWristRollLinkToPoseWithCollisionChecking(tf_pose_in_baselink_new, max_time, wait,planner);
  }

  bool Arm::moveGripperToPoseWithOrientationConstraints(const tf::StampedTransform& tf, 
                                                        bool keep_roll, bool keep_pitch, bool keep_yaw, 
                                                        double max_time, bool wait, double tolerance) {
    tf::StampedTransform tf_new(gripperToWrist(tf));
    return moveWristRollLinkToPoseWithOrientationConstraints(tf_new, keep_roll, keep_pitch, keep_yaw, max_time, wait);
  }

  bool Arm::moveWristRollLinkToPoseWithOrientationConstraints(const tf::StampedTransform& tf, 
                                                              bool keep_roll, bool keep_pitch, bool keep_yaw, 
                                                              double max_time, bool wait, double tolerance) {
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);
    tf::poseStampedTFToMsg(tf_pose,pose);
    return moveWristRollLinkToPoseWithOrientationConstraints(pose, keep_roll, keep_pitch, keep_yaw, 
                                                             max_time, wait, tolerance);
  }

  bool Arm::moveWristRollLinkToPoseWithOrientationConstraints(const geometry_msgs::PoseStamped& pose, 
                                                              bool keep_roll, bool keep_pitch, bool keep_yaw, 
                                                              double max_time, bool wait, double tolerance) {
    if (!move_arm_client_) {
      ROS_ERROR("collision checking arm server has not been started");
      return false;
    }

    moveit_msgs::MoveGroupGoal goal;

    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    goal.request.group_name = group_name;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = max_time;

    // TODO: specify which planner?
    goal.request.planner_id = std::string("");

    // Add goal constraints, just like for ToPoseWithCollisionChecking
    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0].position_constraints.resize(1);
    goal.request.goal_constraints[0].orientation_constraints.resize(1);

    goal.request.goal_constraints[0].position_constraints[0].header.frame_id = pose.header.frame_id;
    goal.request.goal_constraints[0].position_constraints[0].link_name = armside_str + "_wrist_roll_link";
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives.resize(1);
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].type = 2; //SolidPrimitive.SPHERE
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.push_back(0.0001);
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.resize(1);
    goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position = pose.pose.position;

    goal.request.goal_constraints[0].orientation_constraints[0].header.frame_id = pose.header.frame_id;
    goal.request.goal_constraints[0].orientation_constraints[0].orientation = pose.pose.orientation;
    goal.request.goal_constraints[0].orientation_constraints[0].link_name = armside_str + "_wrist_roll_link";
    goal.request.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance = 0.001;
    goal.request.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance = 0.001;
    goal.request.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance = 0.001;
    goal.request.goal_constraints[0].orientation_constraints[0].weight = 1.0;

    // now, add the path_constraints ...
    goal.request.path_constraints.orientation_constraints.resize(1);
    goal.request.path_constraints.orientation_constraints[0].header.frame_id = pose.header.frame_id ;
    goal.request.path_constraints.orientation_constraints[0].link_name = armside_str + "_wrist_roll_link";

    goal.request.path_constraints.orientation_constraints[0].orientation = pose.pose.orientation;
    goal.request.path_constraints.orientation_constraints[0].weight = 1.0;

    if (keep_roll) {
      goal.request.path_constraints.orientation_constraints[0].absolute_x_axis_tolerance = tolerance;
    } else {
      goal.request.path_constraints.orientation_constraints[0].absolute_x_axis_tolerance = M_PI;
    }
    if (keep_pitch) {
      goal.request.path_constraints.orientation_constraints[0].absolute_y_axis_tolerance = tolerance;
    } else {
      goal.request.path_constraints.orientation_constraints[0].absolute_y_axis_tolerance = M_PI;
    }

    if (keep_yaw) {
      goal.request.path_constraints.orientation_constraints[0].absolute_z_axis_tolerance = tolerance;
    } else {
      goal.request.path_constraints.orientation_constraints[0].absolute_z_axis_tolerance = M_PI;
    }

    move_arm_client_->sendGoal(goal);

    bool finished_before_timeout = true;
    if (wait) {
      finished_before_timeout = move_arm_client_->waitForResult(ros::Duration(60.0));
      if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = move_arm_client_->getState();
        //ROS_INFO("Action finished: %s",state.toString().c_str());
      } else {
        //ROS_INFO("Action did not finish before the time out.");
        return false;
      }
    }

    ////		return finished_before_timeout;
    if (move_arm_client_->getState() == actionlib::SimpleClientGoalState::ABORTED) {
      return false;
    } else {
      return true;
    }
  }
} //end namespace simple_robot_control
