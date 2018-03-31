#!/usr/bin/env python
# -*- coding: sjis -*-

import rospy
import numpy as np
from sensor_msgs.msg import Joy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import tf2_ros
from geometry_msgs.msg import WrenchStamped

class PublishHandState(object):
    def __init__(self):
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf)
        self._wrench_com = 0.0
        self._wrench_x = 0.0
        self._wrench_y = 0.0
        self._wrench_z = 0.0
        self._wrench_com_c = 0.0
        self._wrench_x_c = 0.0
        self._wrench_y_c = 0.0
        self._wrench_z_c = 0.0
        self._hand = 0.0
        rospy.Subscriber("/hsrb/wrist_wrench/raw", WrenchStamped, self.cb_wrench_compensated,queue_size=10)
        rospy.Subscriber("/hsrb/wrist_wrench/compensated", WrenchStamped, self.cb_wrench_comp,queue_size=10)

        self.sub = rospy.Subscriber("/hsrb_07/joy", Joy, self.state_set, queue_size=10)
        
#        self.sub = rospy.Subscriber("/hsrb/joint_states", JointState, self.state_set, queue_size=10)
        self.pub = rospy.Publisher("/hsr7/hand_states", JointState, queue_size=10)
#        self.pub_vision_info = rospy.Publisher("/hsr7/object_pos", Point, queue_size=10)
        self.old = None
        self.data = []
#        plt.ion()
#        plt.figure()
        self.rate = rospy.Rate(1)
    def cb_wrench_compensated(self, data):
        dd = data
        x = dd.wrench.force.x
        y = dd.wrench.force.y
        z = dd.wrench.force.z
        self._wrench_x = x
        self._wrench_y = y
        self._wrench_z = z
        self._wrench_com = np.sqrt(np.power(x,2)+np.power(y,2)+np.power(z,2))
    def cb_wrench_comp(self, data):
        dd = data
        x = dd.wrench.force.x
        y = dd.wrench.force.y
        z = dd.wrench.force.z
        self._wrench_x_c = x
        self._wrench_y_c = y
        self._wrench_z_c = z
        self._wrench_com_c = np.sqrt(np.power(x,2)+np.power(y,2)+np.power(z,2))
        

    def state_set(self,data):
        joint = JointState()
        
#        map2hand = self.buf.lookup_transform_full()
        try:
            map2hand = self.buf.lookup_transform("odom", "hand_palm_link",rospy.Time(0),rospy.Duration(3.0))
#            map2hand = self.buf.lookup_transform_full("odom",data.header.stamp, "hand_palm_link",data.header.stamp, "odom")
        except:
            return False
        if data.buttons[1] == 1:            
            self._hand = 0
            print "close"
        elif data.buttons[2] == 1:
            self._hand = 1
            print "open"
        hand_state = self._hand
#        if hand > 1.0:
#            hand_state = 1
#        else:
#            hand_state = 0
#        if hand_state ==1 and self.old ==0:
#            point.x, point.y, point.z = map2hand.transform.translation.x, map2hand.transform.translation.y, map2hand.transform.translation.z
#            self.pub_vision_info.publish(point)
        
        position = [map2hand.transform.translation.x, map2hand.transform.translation.y, map2hand.transform.translation.z, hand_state,
                    map2hand.transform.rotation.x, map2hand.transform.rotation.y, map2hand.transform.rotation.z, map2hand.transform.rotation.w,
                    self._wrench_x, self._wrench_y, self._wrench_z, self._wrench_com,
                    self._wrench_x_c, self._wrench_y_c, self._wrench_z_c, self._wrench_com_c]
        joint.name = ["x", "y", "z", "hand", "qx", "qy", "qz", "qw",
                      "power_x_raw", "power_y_raw","power_z_raw", "power_raw",
                      "power_x", "power_y","power_z", "power"]
        joint.position = position
        joint.effort = [0.,] * len(position)
        joint.velocity = [0.,] * len(position)
        joint.header.stamp = rospy.Time.now()
        self.pub.publish(joint)
        self.old = hand_state
#        self.data.append(position)

    def run(self,):
        while not rospy.is_shutdown():
            pass
#            if len(self.data) > 0:
#                t = np.linspace(0,len(self.data),len(self.data))
#                pos = np.array(self.data).T
#                x = np.array(pos[0])
#                y = np.array(pos[1])
#                z = np.array(pos[2])
#                h = np.array(pos[3])
#
#                plt.subplot(4,1,1)
#                a = plt.plot(range(len(x)),x,"r-")
#                plt.ylabel("x")
#                if len(t)< 100:
#                    plt.xlim(0,100)
#                    plt.ylim(x.min()-0.1,x.max()+0.1)
#                else:
#                    plt.xlim(len(t)-100,len(t))
#                    plt.ylim(x[len(t)-100:len(t)-1].min()-0.1,x[len(t)-100:len(t)-1].max()+0.1)
#                    
#                plt.subplot(4,1,2)
#                b = plt.plot(range(len(y)),y,"g-")
#                plt.ylabel("y")
#                plt.ylim(y.min()-0.5,y.max() + 0.5)
#                if len(t)< 100:
#                    plt.xlim(0,100)
#                    plt.ylim(y.min()-0.5,y.max()+0.5)
#                else:
#                    plt.xlim(len(t)-100,len(t))
#                    plt.ylim(y[len(t)-100:len(t)-1].min()-0.1,y[len(t)-100:len(t)-1].max()+0.1)
#
#                plt.subplot(4,1,3)
#                plt.ylabel("z")
#                c = plt.plot(range(len(z)),z,"b-")
#                plt.ylim(z.min()-0.5,z.max()+0.5)
#                if len(t)< 100:
#                    plt.xlim(0,100)
#                    plt.ylim(z.min()-0.5,z.max()+0.5)
#                else:
#                    plt.xlim(len(t)-100,len(t))
#                    plt.ylim(z[len(t)-100:len(t)-1].min()-0.1,z[len(t)-100:len(t)-1].max()+0.1)
#
#                plt.subplot(4,1,4)
#                d = plt.plot(range(len(h)),h,"c-")
#                plt.ylabel("hand")
#                plt.ylim(-0.1,1.1)
#                if len(t)< 100:
#                    plt.xlim(0,100)
#                else:
#                    plt.xlim(len(t)-100,len(t))
#
#                plt.pause(.01)
        
if __name__ == "__main__":
    rospy.init_node("handstate_publish")
    a = PublishHandState()
    a.run()
    rospy.spin()
