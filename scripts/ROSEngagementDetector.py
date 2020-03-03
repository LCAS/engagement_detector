
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from engagement_detector import EngagementDetector
from TimeSerieInImage import TimeSerieInImage


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
        
        
    def _img_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e

        self.last_img = np.asarray(cv_image)

        if self.plot_in_image:
            out_img = self.image_plotter.step(self.last_img.copy(), self.last_value)
            try:
                out_imgmsg = self.bridge.cv2_to_imgmsg(out_img, "bgr8")
            except CvBridgeError, e:
                print e
            
            self.outImg_pub.publish(out_imgmsg)

    def spin(self, hz=10):

        rate = rospy.Rate(hz)

        rospy.loginfo("Waiting to receive image...")
        while self.last_img is None and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("DONE")

        image_seq = []
        while not rospy.is_shutdown():
            image_seq.append(self.last_img.copy())

            if len(image_seq) > 10:
                image_seq.pop(0) 
                value = np.squeeze(self.detector.predict(image_seq))
                self.eng_pub.publish(value)
                self.last_value = value

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("engagement_detector")

    image = rospy.get_param("~image", "/camera/color/image_raw")
    red = ROSEngagementDetector(image_topic=image)


    red.spin()