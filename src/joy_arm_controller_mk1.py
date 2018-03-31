#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
import actionlib

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Linear_controll(object):
    def __init__(self):
        self.rate = rospy.Rate(10)

#   action
        self._action = actionlib.SimpleActionClient('/hsrb/arm_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self._pub = rospy.Publisher("/hsrb/arm_trajectory_controller/command",JointTrajectory,queue_size=10)

# param set
        self.arm_lift = 0.0
        self.arm_flex = 0.0
        self.arm_roll = 0.0
        self.wrist_flex = -1.57
        self.wrist_roll = 0.0

        self.goal = FollowJointTrajectoryGoal()
        self.traj = JointTrajectory()
        self.traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.p = JointTrajectoryPoint()
        self.p.positions = [self.arm_lift, self.arm_flex, self.arm_roll, self.wrist_flex, self.wrist_roll]
        self.p.velocities = []
        self.p.effort = []
        for i in range(5):
            self.p.velocities += [0.0]
            self.p.effort += [0.0]
        self.p.time_from_start = rospy.Time(1.0)
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
        
        if data.buttons[9] == 1:
            self.wrist_roll += - data.axes[0] * 0.02
            self.wrist_flex += data.axes[1]  * 0.01
            self.arm_flex += data.axes[3] * 0.01
            self.arm_roll += -data.axes[2] * 0.02
            self.arm_lift += float(data.buttons[4]) * 0.001
            self.arm_lift += - float(data.buttons[6]) * 0.001
            self.p.time_from_start = rospy.Time(1.0)
        elif data.buttons[3] == 1:
            self.wrist_roll = 0.0
            self.wrist_flex = -1.57
            self.arm_flex = 0.0
            self.arm_roll = 0.0
            self.arm_lift = 0.0
            self.p.time_from_start = rospy.Time(5.0)

#        elif data.buttons[0] == 1:
#            self.wrist_roll = 0.0
#            self.wrist_flex = 0.0
#            self.arm_flex = -2.6
#            self.arm_roll = 0.0
#            self.arm_lift = 0.6
#            self.p.time_from_start = rospy.Time(5.0)


        
        if self.arm_lift > 0.69:
            self.arm_lift = 0.69
        elif self.arm_lift < 0.0:
            self.arm_lift = 0.0
            
        if self.arm_flex > 0.0:
            self.arm_flex = 0.0
        elif self.arm_flex < -2.6:
            self.arm_flex = -2.6
            
        if self.arm_roll > 3.6:
            self.arm_roll = 3.6
        elif self.arm_roll < -1.9:
            self.arm_roll = -1.9

        if self.wrist_flex > 1.2:
            self.wrist_flex = 1.2
        elif self.wrist_flex < -1.9:
            self.wrist_flex = -1.9

        if self.wrist_roll > 3.6:
            self.wrist_roll = 3.6
        elif self.wrist_roll < -1.9:
            self.wrist_roll = -1.9

        arm_lift = self.arm_lift
        arm_roll = self.arm_roll
        arm_flex = self.arm_flex
        wrist_flex = self.wrist_flex
        wrist_roll = self.wrist_roll            
            
        if data.buttons[9] == 1 or data.buttons[3] == 1 or data.buttons[0] == 1:
            self.p.positions = [arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll]
            self.traj.points = [self.p]            
            self.goal.trajectory = self.traj
            rospy.loginfo(self.goal)
            self._pub.publish(self.traj)
#            self._action.send_goal(self.goal)


if __name__ == '__main__':
    rospy.init_node('arm_controller')
    a = Linear_controll()
    rospy.spin()
