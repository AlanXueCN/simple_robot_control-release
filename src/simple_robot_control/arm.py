#!/usr/bin/env python
#
# Copyright (c) 2010, Bosch LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Bosch LLC nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Sebastian Haug, Bosch LLC
#          Christian Bersch, Bosch LLC
#          Charles DuHadway, Bosch LLC

"""
  This ARM class can be used to control an arm of the pr2 (without collision
  checking). It uses an inverse kinematics server which must be started (e.g. 
  with the launch file arms_ik.launch) if goToPose(), goToPosition() or
  moveGripperToPose() is used.
    
  For a use case see arm_test.py
"""
import numpy
import copy

import rospy
import actionlib
import tf.transformations

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

class Arm:
    def __init__(self, side):
        if side == 'r':
            self.side = 'r'
            self.move_group = 'right_arm'
            self.ikPath = "pr2_right_arm_kinematics"
        elif side == 'l':
            self.side = 'l'
            self.move_group = 'left_arm'
            self.ikPath = "pr2_left_arm_kinematics"
        else:
            rospy.logerr("Abort. Specify 'l' or 'r' for side!")
            exit(1)
        
        self.controller = side+"_arm_controller"
        self.trajClient = actionlib.SimpleActionClient(self.controller+"/joint_trajectory_action", JointTrajectoryAction)
        
        if not self.trajClient.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Could not connect to /joint_trajectory_action of " + self.side + " arm.")
            exit(1)
        
        self.joint_names = self.getJointNames()
        
        self.runIK = rospy.ServiceProxy("compute_ik", GetPositionIK)


    def cancelTraj(self):
        self.trajClient.cancel_all_goals()
      
    def sendTraj(self, traj, wait = True):
        """ Send the joint trajectory goal to the action server """
        if wait:
          self.trajClient.send_goal_and_wait(traj)
        else:
          self.trajClient.send_goal(traj)
    
    def getJointNames(self):
        """ Contacts param server to get controller joints """
        return rospy.get_param(self.controller + "/joints");

    def getJointAngles(self):
        msg = rospy.wait_for_message("/joint_states", JointState)
        angles = []
        for name in self.joint_names:
          angles.append( msg.position[msg.name.index(name)])
        return angles

    def getJointState(self):
        msg = rospy.wait_for_message("/joint_states", JointState)
        return msg

    
    def getIK(self, goal_pose, group_name, seed_angles):
        """ Calls inverse kinematics service for the arm. 
            goalPose -  Pose Stamped message showing desired position of link in coord system
            group_name - move_group for moveit (left_arm or right_arm)
            startAngles -  seed angles for IK
        """
        ikreq = GetPositionIKRequest()
        ikreq.ik_request.group_name = group_name
        ikreq.ik_request.pose_stamped = goal_pose
        ikreq.ik_request.robot_state.joint_state = seed_angles

        ikres  = self.runIK(ikreq)
        if (ikres.error_code.val != ikres.error_code.SUCCESS): #figure out which error code
            raise rospy.exceptions.ROSException("Could not find IK \n\n" + ikres.__str__())

        # take the whole robot state message and pull out the angles that are needed
        ik_angles = []
        for name in self.joint_names:
            idx = ikres.solution.joint_state.name.index(name)
            ik_angles.append(ikres.solution.joint_state.position[idx])

        return ik_angles

    def goToAngle(self, angles, time, wait = True):
        """ This method moves the arm to the specified joint angles. Users must make
            sure to obey safe joint limits.
        """
        if (len(angles) != len(self.joint_names)):
            raise Exception("Wrong number of Angles. "+ len(angles) + "given. "+ len(self.joint_names) + "needed.")
    
        trajMsg = JointTrajectoryGoal()
        
        trajPoint = JointTrajectoryPoint()
        trajPoint.positions = angles
        trajPoint.velocities = [0 for i in range(0, len(self.joint_names))]
        trajPoint.time_from_start = rospy.Duration(time)
            
        trajMsg.trajectory.joint_names = self.joint_names
        trajMsg.trajectory.points.extend([trajPoint])
        self.sendTraj(trajMsg, wait)
        
    def rotateWrist(self, angle, speed = 2.0,  wait = True, velocity = 0.0):
        """ Rotates wrist by angle in radians. speed is the amount of time that it would take for one revolution.
            Velocity sets explicit speed to be attained  - careful! If velocity is to high, the wrist may rotate in
            opposite direction first
        """
        time = speed * abs(angle) * 3.14 / 2
        trajMsg = JointTrajectoryGoal()
        
        trajPoint = JointTrajectoryPoint()
        angles = self.getJointAngles()
        
        trajPoint.positions = angles
        trajPoint.velocities = [0 for i in range(0, len(self.joint_names) - 1)]
        trajPoint.velocities.append(velocity)
        trajPoint.time_from_start = rospy.Duration(time)
        trajMsg.trajectory.joint_names = self.joint_names
        #set new wrist angle
        dm = divmod(abs(angle),3.14)
        for i in range(int(dm[0])):
            trajPoint.positions[-1] += 3.14 * numpy.sign(angle)
            trajPoint.time_from_start = rospy.Duration(time * (i+1)*3.14 / abs(angle))
            trajMsg.trajectory.points.append(copy.deepcopy(trajPoint))
        trajPoint.positions[-1] += dm[1] * numpy.sign(angle)
        trajPoint.time_from_start = rospy.Duration(time)
        trajMsg.trajectory.points.append(trajPoint) 
        self.sendTraj(trajMsg, wait)
        

    def goToPosition(self, pos, frame_id, time, wait = True):
        """ This method uses getIK to move the x_wrist_roll_link to the 
            specific pos specified in frame_id's coordinates. Time specifies
            the duration of the planned trajectory, wait whether to block or not. 
        """
        self.goToPose(pos, (0, 0, 0, 1), frame_id, time, wait)
        
    def goToPose(self, pos, orientation, frame_id, time, wait = True, seed_angles_list = []):
        """ This method uses getIK to move the x_wrist_roll_link to the 
            specific point and orientation specified in frame_id's coordinates. 
            Time specifies the duration of the planned trajectory, 
            wait whether to block or not. 
        """
        pose = self.makePose(pos, orientation, frame_id)
        if not seed_angles_list:
            seed_angles = self.getJointState()
        else:
            seed_angles = JointState()
            seed_angles.name = self.joint_names
            seed_angles.position = seed_angles_list
        ik_angles  = self.getIK(pose, self.move_group, seed_angles)
        self.goToAngle(ik_angles, time, wait)

    
    def moveGripperToPose(self, pos, orientation, frame_id, time, wait = True, seed_angles_list = []):
        """ This method uses gripperToWrist() to calculate the pos_new of the wrist
            so that the gripper is moved to the given pos and orientation.
        """
        pos_new = self.gripperToWrist(pos, orientation)
        self.goToPose(pos_new, orientation, frame_id, time, wait, seed_angles_list)
       
    def gripperToWrist(self, pos, orientation):
        gripper_length = 0.18
        vec = (gripper_length, 0 , 0 , 1)
        rot = tf.transformations.quaternion_matrix(orientation)
        offset = numpy.dot(rot, vec)
        return pos - offset[0:3]
       
    def makePose(self, xyz, orientation, frameID):
        """ This is a shortcut method for making pose stamped messages
        """
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz[0], xyz[1], xyz[2]
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1] 
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        pose.header.frame_id = frameID
        pose.header.stamp = rospy.Time.now()
        return pose
    
    

        
        
        
        
        
        
        
