#!/usr/bin/env python
# coding: utf-8
from __future__ import unicode_literals, print_function
import rospy
import cv2

from cv_bridge import CvBridge
import sensor_msgs.msg as sensor_msg


class HandCameraRotation(object):
    """
    手先カメラの画像を回転させて送信し直すためのクラス
    """
    def __init__(self):
        self._image = None

        # topics
        self._sub_image = rospy.Subscriber(
            "/hsrb/hand_camera/image_raw", sensor_msg.Image, self._image_cb)
        self._pub_image = rospy.Publisher(
            "/hsr7/hand_camera/rot_image_raw",
            sensor_msg.Image,
            queue_size=1)

    @property
    def image(self):
        return self._image

    def _image_cb(self, img):
        """
        手先カメラからの画像を受け取るコールバック関数。

        手先カメラから送られる画像を半時計回りに回転させる

        Parameters
        ----------
        img: sensor_msg.Image
            手先カメラ画像
        """
        # sensor_msg.Image -> numpy.ndarray
        image = CvBridge().imgmsg_to_cv2(img)
        # 画像を時計回りに回転 (転置を取ることにより実現)
        rot_image = cv2.transpose(image, )
        rot_image = cv2.flip(rot_image, 0)
        # 画像を上下反転
#        rot_image = image[::-1]
        rot_image = rot_image[:, :, ::-1]
        self._image = CvBridge().cv2_to_imgmsg(rot_image)
        self._image.header.stamp = rospy.Time.now()
        self._image.header.frame_id = "hand_palm_link"
        self._image.encoding = "bgr8"

        # 送信
        img
        self._pub_image.publish(self._image)


if __name__ == "__main__":
    rospy.init_node("hand_camera_rotation")

    rotationer = HandCameraRotation()
    name = rospy.get_name()
    rospy.loginfo("start {name}".format(**locals()))
    rospy.spin()
