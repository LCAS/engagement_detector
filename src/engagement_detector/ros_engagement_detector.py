#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from engagement_detector import EngagementDetector
from time_serie_in_image import TimeSerieInImage

from threading import Lock


class ROSEngagementDetector():

    def __init__(self, image_topic, plot_in_image=True, out_image_topic="/engagement_detector/out_image"):
        self.plot_in_image = plot_in_image
        self.bridge = CvBridge()
        self.detector = EngagementDetector()

        rospy.Subscriber(image_topic, Image, self._img_cb)

        self.eng_pub = rospy.Publisher("/engagement_detector/value", Float32, queue_size=1)

        if self.plot_in_image:
            self.image_plotter = TimeSerieInImage()
            self.outImg_pub = rospy.Publisher(out_image_topic, Image, queue_size=1)

        self.last_img = None
        self.last_value = 0.0
        self.image_seq = []

    def _img_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr(e)

        self.last_img = np.asarray(cv_image)

        if self.plot_in_image:
            out_img = self.image_plotter.step(self.last_img.copy(), self.last_value)
            try:
                out_imgmsg = self.bridge.cv2_to_imgmsg(out_img, "bgr8")
            except CvBridgeError, e:
                rospy.logerr(e)

            self.outImg_pub.publish(out_imgmsg)

    def spin(self, hz=10):
        # to ensure that the seuquence has 10 elements in timer thread 
        sequence_lock = Lock()

        # the code to execute every tenth of a second (but it can go slower if prediction is not fast enough)
        def timed_cb(event):

            sequence_lock.acquire()
            prediction = self.detector.predict(self.image_seq)
            sequence_lock.release()
            if prediction is None:
                rospy.logwarn("Could not make a prediction, probably the frame sequence length is not 10.")
                return

            value = np.squeeze(prediction)
            self.eng_pub.publish(value)
            self.last_value = value

        timer = rospy.Timer(rospy.Duration.from_sec(1./hz),  timed_cb, oneshot=False)

        rate = rospy.Rate(hz)

        rospy.loginfo("Waiting to receive image...")
        while self.last_img is None and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("DONE")


        # save images at hz rate
        while not rospy.is_shutdown():
            sequence_lock.acquire()
            self.image_seq.append(self.last_img.copy())
            if len(self.image_seq) > 10:
                self.image_seq.pop(0)
            sequence_lock.release()

            rate.sleep()

        # stop the timer firing
        timer.shutdown()

if __name__ == "__main__":
    rospy.init_node("engagement_detector")

    image = rospy.get_param("~image_topic", "/camera/color/image_raw")
    debug_image = rospy.get_param("~debug_image", True)
    out_image = rospy.get_param("~out_image", "/engagement_detector/out_image")

    red = ROSEngagementDetector(image_topic=image, plot_in_image=debug_image, out_image_topic=out_image)

    red.spin()
