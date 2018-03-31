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
from rosbag_database.srv import RosbagRecord, RosbagRecordRequest
from rosbag_database.srv import RosbagStop, RosbagStopRequest
import os
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import String
from audio_module_msg.msg import AudioSentence
TOPICS=["/hsr7/hand_states",
        "/hsrb/joint_states",
        "/tf",
        "/hsrb/head_rgbd_sensor/rgb/image_raw",
        "/amcl_pose",
        "/AudioSentence",
        "/wlan/compressed",
        "/hsrb_07/joy"
        ]
ENV="environment"        
IMAGE = []
REC = 0.7
TT = 10.0     
class Linear_controll(object):
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.sentencec = "待機中"        
        # topics
#        self.sub_joint = rospy.Subscriber("/hsrb/joint_states", JointState ,self.now_pose,queue_size=10)
        self.sub = rospy.Subscriber("/hsrb_07/joy", Joy, self.callback, queue_size=10)
        rospy.Subscriber("/hsrb/wrist_wrench/compensated", WrenchStamped, self.cb_wrench_compensated,queue_size=10)
        self.grip_act = rospy.Publisher("/hsrb/gripper_controller/grasp/goal",GripperApplyEffortActionGoal, queue_size=10)
        self.pub_list = rospy.Publisher('/hsrb/gripper_controller/command', JointTrajectory, queue_size=10)
        self.sub_obj = rospy.Subscriber("/hsr7/recog_obj/name", String, self.recog_obj,queue_size=10)
#        self.vision_obj = rospy.Subscriber("/ext/vision_module/object_recognition_info", ObjectInfo, self.recog_vision,queue_size=10)
        self.pub_sig = rospy.Publisher('/hsrb/gripper_trajectory_controller/command', JointTrajectory, queue_size=10)
        rospy.Subscriber("/AudioSentence", AudioSentence,self.set_audio,queue_size=10)
        

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
        
        self.srv_record_start = rospy.ServiceProxy("rosbag_record", RosbagRecord)
        self.srv_record_stop = rospy.ServiceProxy("rosbag_record_stop", RosbagStop)
        self.srv_marker = rospy.ServiceProxy("marker_data_saver", Empty)
        self.rosbag_number = 0
        self.record_flag = False
        self.rosbag_time = rospy.Time(0)
        self.record_number = 0
        self.number = 0
        date = time.ctime()
        date = date.replace("  ", " ")
        self.date = date.split(" ")        
        self.image_number = 0
        self.image_time = 0
# TF
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf)
        self._wrench=WrenchStamped()
        self.obj_name = ""
        self.obj_time = rospy.Time.now().to_sec()
        self.vis_obj_name = ""
        self.vis_obj_num = 0
        self.vis_obj_time = rospy.Time.now().to_sec()
        self.flag_record = True
        
    def set_audio(self,data):
        print data.sentences[0]
        self.sentencec = data.sentences[0]
        
    def recog_vision(self,msg):
        objs = msg.objects
        count = 0
        obl = []
        obc = []
        for o in objs:
          if o.specific[0].score < REC:
              continue
          count += 1
          i = o.specific[0].id
          if not i in obl:
              obl.append(i)
              obc.append(1)
          else:
              bl = np.array(obl)
              cc = bl[bl==i][0]
              try:
                  obc[cc]+= 1
              except:
                  pass
        if obl != []:
            print obl
            c = obl[np.argmax(obc)]
            self.vis_obj_name = "obj_id_{}".format(c)
            self.vis_obj_num = count
            self.vis_obj_time = msg.header.stamp.to_sec()
        
    def recog_obj(self,data):
        self.obj_name = data.data
        t = rospy.Time.now().to_sec()
        self.obj_time = t
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
            grip.goal.effort = -0.02
            print "grasp"
            self.grip_act.publish(grip)
            pp = JointTrajectoryPoint()
            pp.positions = [-0.05,-0.05]
            pp.velocities = []
            pp.effort = []
            self.hand_sig.points = [pp]
            self.pub_sig.publish(self.hand_sig)            
            time.sleep(0.1)
            self.hand_flug = False
        elif data.buttons[3] == 1:
            self.flag_record = True
            self.obj_name = ""

## 改良            
        elif data.buttons[2] == 1:
            print "open"
            self.pub_list.publish(self.hand_traj)
            pp = JointTrajectoryPoint()
            pp.positions = [0.61,-0.61]
            pp.velocities = []
            pp.effort = []
            self.hand_sig.points = [pp]
            self.pub_sig.publish(self.hand_sig)            
            time.sleep(0.1)
            self.hand_flug = True

        elif data.buttons[0] == 1:
            self.obj_name = ""
            if not self.record_flag and self.flag_record:
                self.flag_record = False                
                bb_req = RosbagRecordRequest()
                bb_req.node_name= "joint_teleope_bag"
                bb_req.save_name= "exp_{0:s}_{1:d}".format(ENV,self.number)
                bb_req.split_duration_str = "60"
                bb_req.record_topic_list = TOPICS
                self.rosbag_time = rospy.Time.now()
                self.record_number = 0
                self.record_flag = True
                res = self.srv_record_start.call(bb_req)
            elif self.flag_record and self.flag_record:
                bs_req = RosbagStopRequest()
                bs_req.rosbag_number = self.record_number
                self.record_flag = False
                self.number += 1
                self.flag_record = False                
                self.srv_record_stop.call(bs_req)
            rospy.sleep(1.0)
        elif data.buttons[16] == 1:
            st = EmptyRequest()
            a = self.srv_marker.call(st)
            


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
        print "Audio: {0:s} ".format(self.sentencec)
        if (rospy.Time.now().to_sec() - self.vis_obj_time) <= TT:
            print "Recog : {0:s} : Num: {1:d} : Time : {2:f}".format(self.vis_obj_name,self.vis_obj_num, (self.vis_obj_time - rospy.Time.now().to_sec()))
        else:
            print "Recog : No"
        if (rospy.Time.now().to_sec() - self.obj_time) <= TT:
            print "Found : {0:s}".format(self.obj_name)
        else:
            print "Found : No"
        print "Nomber:{}".format(self.number)
        if self.hand_flug:
            print "hand:open"
        else:
            print "hand:close"
        w = self.wrench_calc()
        print "pressure : ", w
        if w > 30.0:
            print "too much power"

        if self.record_flag:
            print"now save joint and key"
            print rospy.Time.now().to_sec() - self.rosbag_time.to_sec()                

if __name__ == '__main__':
    rospy.init_node('joy_interface')
    a = Linear_controll()
    a.run()
