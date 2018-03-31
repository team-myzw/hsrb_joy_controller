#!/usr/bin/env python
# -*- coding: sjis -*-

#import os
import rospy
import numpy as np
import pandas as pd
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
import tf
from std_srvs.srv import Empty

CSV = "data_frame/data_frame_{}_.csv"
NAME= "exp"
BASE= "odom"
TIME = 3.0
DIS = 2.0
import os
class MarkerSaver(object):
    def __init__(self):
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf)
        self.set_state()
        self.num = 0
        rospy.Service("marker_data_saver", Empty, self.save_signal)
        try:
            data = pd.read_csv(CSV)
            ids = data.id.values[-1] +1
            self.num = ids
        except:
            rospy.loginfo("no csv file")
        self.key = True
        rospy.loginfo("start record")
        self.sub = rospy.Subscriber("/hsrb/joint_states", JointState, self.state_set, queue_size=10)
        self.pub = rospy.Publisher("/hsr7/recog_obj/name", String, queue_size=10)
    def set_state(self,):        
        self.rostime = []
        self.pos_x = []
        self.pos_y = []
        self.pos_z = []
        self.szwht_x = []
        self.szwht_y = []
        self.szwht_z = []
        self.generic_id = []
        self.generic_name = []
        self.generic_score = []
        self.ids = []
        self.switch=False

    def save_signal(self, data):
        rospy.loginfo("get signal")
        self.switch = True
        rospy.sleep(3.0)
        return None
        
    def state_set(self,data):
        ss = data.header.stamp
        if self.key:
            tf_list = self.buf.all_frames_as_string()
            ts = tf_list.split("\n")
            mark_list = []
            for s in ts:
                k = s.split(" ")
                if len(k) > 1:
                    if NAME in k[1]:
                        mark_list.append(k[1])
            for mark in mark_list:
                try:
                    data = self.buf.lookup_transform_full(BASE,ss,mark, ss,BASE,rospy.Duration(TIME))
                    t = rospy.Time.now()
                    self.rostime.append(int(t.to_nsec()))
                    self.pos_x.append(data.transform.translation.x)            
                    self.pos_y.append(data.transform.translation.y)            
                    self.pos_z.append(data.transform.translation.z) 
                    xyz = tf.transformations.euler_from_quaternion([data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w])
                    self.szwht_x.append(xyz[0])
                    self.szwht_y.append(xyz[1])
                    self.szwht_z.append(xyz[2])
                    names = mark.split("_")                    
                    self.generic_id.append(int(names[-1]))
                    self.generic_name.append(mark)
                    self.generic_score.append(1.0)
                    self.ids.append(self.num)
                    self.num += 1
                    st = String()
                    st.data = mark
                    self.pub.publish(st)
                    rospy.logwarn("get frame {}".format(mark))
                except:
                    rospy.loginfo("Not frame {}".format(mark))
                    self.buf = tf2_ros.Buffer()
                    self.lis = tf2_ros.TransformListener(self.buf)
                
    def save(self):
        rospy.loginfo("check csv file")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                if self.switch:
                    break
                rate.sleep()
    
            df = pd.DataFrame({"id": np.array(self.ids,dtype=np.int),
                               "position_x": np.array(self.pos_x, dtype=np.float),
                               "position_y": np.array(self.pos_y, dtype=np.float),
                               "position_z": np.array(self.pos_z, dtype=np.float),
                               "szwht_x": np.array(self.szwht_x, dtype=np.float),
                               "szwht_y": np.array(self.szwht_y, dtype=np.float),
                               "szwht_z": np.array(self.szwht_z, dtype=np.float),
                               "generic_id_0": np.array(self.generic_id, dtype=np.int),
                               "generic_name_0": np.array(self.generic_name, dtype=np.str),
                               "generic_score_0": np.array(self.generic_score, dtype=np.float),
                               "ros_timestamp": np.array(self.rostime, dtype=np.float)
                               })           
            df.to_csv(CSV.format(os.times()[-1]))
            rospy.loginfo("finish to save marker data to csv file")
            rospy.loginfo("save csv")
            self.set_state()

if __name__ == "__main__":
    rospy.init_node("marker_saver")
    a = MarkerSaver()
    rospy.loginfo("now recording marker data")
    rospy.loginfo("please push enter")
    rospy.loginfo("finish recording marker data")
    rospy.sleep(0.5)
    rospy.loginfo("please wait for marker date to csv file")
    a.save()
