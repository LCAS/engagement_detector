#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from engagement_detector.engagement_detector import EngagementDetector
from engagement_detector.time_serie_in_image import TimeSerieInImage
from threading import Lock


class ROSEngagementDetector(Node):

    def __init__(self):
        super().__init__('engagement_detector')
        image_topic = self.declare_parameter("image", "/camera/color/image_raw").value
        plot_in_image = self.declare_parameter("debug_image", True).value
        out_image_topic = self.declare_parameter("out_image", "engagement_detector/out_image").value

        self.plot_in_image = plot_in_image
        self.bridge = CvBridge()
        self.detector = EngagementDetector()

        self.subscription = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self._img_cb,
            10
        )
        self.subscription  # prevent unused variable warning

        self.eng_pub = self.create_publisher(Float32, 'engagement_detector/value', 1)

        if self.plot_in_image:
            self.image_plotter = TimeSerieInImage()
            self.outImg_pub = self.create_publisher(Image, out_image_topic, 1)

        self.last_img = None
        self.last_value = 0.0
        self.image_seq = []
        # to ensure that the sequence has 10 elements in timer thread
        self.sequence_lock = Lock()

    def _img_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)

        last_img = np.asarray(cv_image)

        self.sequence_lock.acquire()
        self.image_seq.append(last_img.copy())
        if len(self.image_seq) > 10:
            self.image_seq.pop(0)
        self.sequence_lock.release()

        if self.plot_in_image:
            out_img = self.image_plotter.step(last_img.copy(), self.last_value)
            try:
                out_imgmsg = self.bridge.cv2_to_imgmsg(out_img, "bgr8")
            except CvBridgeError as e:
                self.get_logger().error(e)

            self.outImg_pub.publish(out_imgmsg)

    def timed_cb(self):
        if len(self.image_seq) < 10:
            self.get_logger().warn("Waiting to receive images...")
            return

        self.sequence_lock.acquire()
        tmp_image_seq = self.image_seq
        self.sequence_lock.release()
        prediction = self.detector.predict(tmp_image_seq)
        if prediction is None:
            self.get_logger().warn("Could not make a prediction, probably the frame sequence length is not 10.")
            return

        value = np.squeeze(prediction)[()].item()
        fmsg = Float32()
        fmsg.data = value
        self.eng_pub.publish(fmsg)
        self.last_value = value

    def spin(self, hz=10):
        timer = self.create_timer(1.0 / hz, self.timed_cb)

        rclpy.spin(self)

        # stop the timer firing
        timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    red = ROSEngagementDetector()
    # Start the engagement detector
    red.spin()

    # Shutdown the ROS2 node
    rclpy.shutdown()



if __name__ == '__main__':
    main()
