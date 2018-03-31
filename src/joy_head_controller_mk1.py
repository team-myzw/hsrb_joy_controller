#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
import time
import actionlib

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Linear_controll(object):
    def __init__(self):
        self.rate = rospy.Rate(10)

#   action
        self._action = actionlib.SimpleActionClient('/hsrb/head_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self._pub = rospy.Publisher("/hsrb/head_trajectory_controller/command",JointTrajectory, queue_size=10)
# param set
        self.head_pan = 0.0
        self.head_tilt = 0.0
    
        self.goal = FollowJointTrajectoryGoal()
        self.traj = JointTrajectory()
        self.traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        self.p = JointTrajectoryPoint()
        self.p.positions = [self.head_pan, self.head_tilt]
        self.p.velocities = []
        for i in range(2):
            self.p.velocities += [0.0]
        self.p.time_from_start = rospy.Time(1)
        self.traj.points = [self.p]

        self.goal.trajectory = self.traj
        print("send_acition")
        self._action.send_goal(self.goal)
        # topics
#        self.sub_joint = rospy.Subscriber("/hsrb/joint_states", JointState ,self.now_pose,queue_size=10)
        self.sub = rospy.Subscriber("/hsrb_07/joy", Joy, self.callback, queue_size=10)
        


    def now_pose(self,data):
        self.now_arm_lift = data.position[0]
        self.now_command = data.position[1]
        self.now_arm_roll = data.position[2]
        self.now_pose_command = data.position[11]
        self.now_wrist_command = data.position[12]
        self.now_head_pan_command = data.position[9]
        self.now_head_tilt_command = data.position[10]
        
    def callback(self,data):
        
        if data.buttons[11] == 1:
            self.head_pan += data.axes[0] * 0.025
            self.head_tilt += data.axes[1]  * 0.025
            self.p.time_from_start = rospy.Time(1.0)
        elif data.buttons[3] == 1:
            self.head_pan = 0.0
            self.head_tilt = 0.0
            self.p.time_from_start = rospy.Time(5.0)
        
        if self.head_pan > 1.7:
            self.head_pan = 1.7
        elif self.head_pan < -3.8:
            self.head_pan = -3.8
            
        if self.head_tilt > 0.5:
            self.head_tilt = 0.5
        elif self.head_tilt < -1.57:
            self.head_tilt = -1.57

        head_pan = self.head_pan
        head_tilt = self.head_tilt


        if data.buttons[11] == 1 or data.buttons[3] == 1:
            self.p.positions = [head_pan, head_tilt]
            self.traj.points = [self.p]
            self._pub.publish(self.traj)
            self.goal.trajectory = self.traj
#            self._action.send_goal(self.goal)


if __name__ == '__main__':
    rospy.init_node('head_controller')
    a = Linear_controll()
    rospy.spin()
