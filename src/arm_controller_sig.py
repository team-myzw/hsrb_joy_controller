#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryResult, FollowJointTrajectoryActionFeedback
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tmc_control_msgs.msg import GripperApplyEffortActionGoal
TIME=0.0



class HandGrip(object):
    def __init__(self):
        rospy.Subscriber("/hsrb/joint_states", JointState, self.cb_joint, queue_size=10)
        rospy.set_param('/hsrb/arm_trajectory_controller/joints', ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"])
        self.action_server = actionlib.SimpleActionServer("/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
                                                    FollowJointTrajectoryAction, execute_cb=self.cb,
                                                    auto_start=False)
        self.grip_act = rospy.Publisher("/hsrb/arm_trajectory_controller/command",JointTrajectory,queue_size=10)
        
        self.rate = rospy.Rate(100)
        self.hand_state = 0.0
        self.action_server.start()

    def cb_joint(self, msg):
        self.hand_state = msg.position[0]
                

    def cb(self, data):
        """
        指定時間の間に閾値以上の力が力各センサに加わったかどうかを返す。
        現在の値からの変位を見ているため、開始のタイミングに注意

        Args:
            ntime  int32: 指定時間
            nforce float64: 閾値の力
        Result:
            success bool:
        Feedback:
            force float64: 現在加えられている力
        """
        num = 1
        N = len(data.trajectory.points)

        success =FollowJointTrajectoryResult()
        success.SUCCESSFUL
        end_sec = data.trajectory.points[-1].time_from_start.to_sec()
        start_sec = rospy.Time.now().to_sec()
        now = rospy.Time(0)
        joint = JointTrajectory()
        joint.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        j = JointTrajectoryPoint()
        j.positions = data.trajectory.points
        j.time_from_start = data.trajectory.points[0].time_from_start - now
        now = data.trajectory.points[0].time_from_start
        joint.points = [j]
        self.grip_act.publish(joint)
        while (rospy.Time.now().to_sec()-start_sec) <= (end_sec + 1.0) and not rospy.is_shutdown():
            if (rospy.Time.now().to_sec()-start_sec) >= data.trajectory.points[num-1].time_from_start.to_sec()-TIME:
                j = JointTrajectoryPoint()
                j.positions = data.trajectory.points
                j.time_from_start = data.trajectory.points[num].time_from_start - now
                now = data.trajectory.points[0].time_from_start
                joint.points = [j]
                self.grip_act.publish(joint)                
                num += 1
            if num == N:
                break
#            self.action_server.publish_feedback(feedback)
            self.rate.sleep()
        self.action_server.set_succeeded(success)


if __name__ == "__main__":
    rospy.init_node("arm_state_controller")
    wrist_wrench = HandGrip()
    rospy.spin()    
