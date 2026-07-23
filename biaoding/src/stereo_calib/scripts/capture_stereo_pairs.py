#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目相机同步图像对采集工具

用法:
    rosrun stereo_calib capture_stereo_pairs.py
    # 或自定义话题:
    rosrun stereo_calib capture_stereo_pairs.py \
        _left_topic:=/hik_camera/left/image \
        _right_topic:=/hik_camera/right/image

交互:
    [空格] 同步保存一对图像
    [ q ]  退出
"""
import os
import sys
import rospy
import cv2
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class StereoCapture:
    def __init__(self):
        rospy.init_node("stereo_capture", anonymous=False)

        # 参数
        default_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "images"
        )
        self.left_topic = rospy.get_param("~left_topic", "/hik_camera/left/image")
        self.right_topic = rospy.get_param("~right_topic", "/hik_camera/right/image")
        self.save_dir = rospy.get_param("~save_dir", default_dir)
        self.slop = rospy.get_param("~slop", 0.05)
        self.show_scale = rospy.get_param("~show_scale", 0.5)

        os.makedirs(self.save_dir, exist_ok=True)

        # 统计已有图像，避免覆盖
        exist = [
            f for f in os.listdir(self.save_dir)
            if f.startswith("left_") and f.endswith(".png")
        ]
        self.count = len(exist)

        self.bridge = CvBridge()
        self.left_img = None
        self.right_img = None
        self.latest_header = None

        # 订阅 + 近似时间同步
        left_sub = message_filters.Subscriber(self.left_topic, Image)
        right_sub = message_filters.Subscriber(self.right_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=self.slop
        )
        self.sync.registerCallback(self.image_callback)

        rospy.loginfo(f"[采集] 左话题: {self.left_topic}")
        rospy.loginfo(f"[采集] 右话题: {self.right_topic}")
        rospy.loginfo(f"[采集] 保存目录: {self.save_dir}")
        rospy.loginfo(f"[采集] 已存在 {self.count} 对, 新采集将从 {self.count:03d} 号起累加")

        # 显示定时器 30Hz
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.show_callback)

    def image_callback(self, left_msg, right_msg):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            self.right_img = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")
            self.latest_header = left_msg.header
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, f"[采集] cv_bridge 转换失败: {e}")

    def show_callback(self, _event):
        if self.left_img is None or self.right_img is None:
            return

        scale = self.show_scale
        left_small = cv2.resize(self.left_img, (0, 0), fx=scale, fy=scale)
        right_small = cv2.resize(self.right_img, (0, 0), fx=scale, fy=scale)
        combined = cv2.hconcat([left_small, right_small])

        hint = f"[saved: {self.count}]   [space]=save   [q]=quit"
        cv2.putText(
            combined, hint, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        cv2.imshow("stereo_capture (left | right)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            self.save_pair()
        elif key == ord("q") or key == 27:  # q or ESC
            rospy.signal_shutdown("user quit")

    def save_pair(self):
        idx = self.count
        left_path = os.path.join(self.save_dir, f"left_{idx:03d}.png")
        right_path = os.path.join(self.save_dir, f"right_{idx:03d}.png")
        cv2.imwrite(left_path, self.left_img)
        cv2.imwrite(right_path, self.right_img)
        self.count += 1
        rospy.loginfo(f"[采集] 保存 #{idx:03d} -> {left_path}")


def main():
    try:
        StereoCapture()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
