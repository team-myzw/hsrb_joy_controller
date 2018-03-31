#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
import time
import tf2_ros
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped

import os


class Linear_controll(object):
    def __init__(self):
        self.rate = rospy.Rate(10)
        # topics
#        self.sub_joint = rospy.Subscriber("/hsrb/joint_states", JointState ,self.now_pose,queue_size=10)
        self.sub = rospy.Subscriber("/hsrb_07/joy", Joy, self.callback, queue_size=10)
        rospy.Subscriber("/hsrb/wrist_wrench/compensated", WrenchStamped, self.cb_wrench_compensated,queue_size=10)
        self.grip_act = rospy.Publisher("/hsrb/gripper_controller/grasp/goal",GripperApplyEffortActionGoal, queue_size=10)
        self.pub_list = rospy.Publisher('/hsrb/gripper_controller/command', JointTrajectory, queue_size=10)
        self.pub_sig = rospy.Publisher('/hsrb/gripper_trajectory_controller/command', JointTrajectory, queue_size=10)
# service
#
        self.hand_traj = JointTrajectory()
        self.hand_sig = JointTrajectory()
        self.hand_traj.joint_names = ["hand_motor_joint"]
        self.hand_sig.joint_names = ["hand_l_proximal_joint","hand_r_proximal_joint"]
        pp = JointTrajectoryPoint()
        pp.positions = [1.2,]
        pp.velocities = [0]
        pp.effort = [0.1]
        pp.time_from_start = rospy.Time(3)
        self.hand_traj.points = [pp]
        self.hand_sig.points = [pp]
        self.hand_flug = True
# TF
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf)
        self._wrench=WrenchStamped()
    def cb_wrench_compensated(self,cb):
        self._wrench = cb


    def now_pose(self,data):
        self.now_command = data.position[1]
        self.now_arm_command = data.position[0]
        self.now_arm_roll = data.position[2]
        self.now_pose_command = data.position[11]
        self.now_wrist_command = data.position[12]
        self.now_head_pan_command = data.position[9]
        self.now_head_tilt_command = data.position[10]
        
    def callback(self,data):


        if data.buttons[1] == 1:
            grip = GripperApplyEffortActionGoal()
            grip.goal.effort = -0.05
            print "grasp"
#            self.grip_act.publish(grip)
            pp = JointTrajectoryPoint()
            pp.positions = [-0.05,-0.05]
            pp.velocities = []
            pp.effort = []
            self.hand_sig.points = [pp]
            self.pub_sig.publish(self.hand_sig)            
            
            time.sleep(0.1)
            self.hand_flug = False

## 改良            
        elif data.buttons[2] == 1:
            print "open"
 #           self.pub_list.publish(self.hand_traj)
            pp = JointTrajectoryPoint()
            pp.positions = [0.61,-0.61]
            pp.velocities = []
            pp.effort = []
            self.hand_sig.points = [pp]
            self.pub_sig.publish(self.hand_sig)            
            time.sleep(0.1)
            self.hand_flug = True



    def wrench_calc(self,):
        wrench = self._wrench
        f = wrench.wrench.force
        f_a = np.sqrt(f.x**2 + f.y**2 + f.z**2)
        return f_a
        

    def run(self):
        while not rospy.is_shutdown():
            self.show_interface()
            self.rate.sleep()
        
    def show_interface(self, ):
        os.system("clear")
        if self.hand_flug:
            print "hand:open"
        else:
            print "hand:close"
        w = self.wrench_calc()
        print "pressure : ", w
        if w > 30.0:
            print "too much power"

                

if __name__ == '__main__':
    rospy.init_node('joy_interface')
    a = Linear_controll()
    a.run()
