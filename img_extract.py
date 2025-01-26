import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
 
class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_count = 0
        self.depth_count = 0
        self.rgb_images = []
        self.depth_images = []
 
        rospy.init_node('image_saver', anonymous=True)
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
 
    def rgb_callback(self, msg):
        rospy.loginfo(f'Callback')
        if self.rgb_count < 1:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.rgb_images.append(cv_image)
            cv2.imwrite(f'rgb_image_8.jpg', cv_image)
            self.rgb_count += 1
            rospy.loginfo(f'Saved rgb_image_{self.rgb_count}.jpg')
 
    def depth_callback(self, msg):
        if self.depth_count < 1:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_images.append(depth_img)
            np.save(f'depth_image_8.npy', depth_img)
            self.depth_count += 1
            rospy.loginfo(f'Saved depth_image_{self.depth_count}.npy')
 
    def run(self):
        rospy.spin()
 
if __name__ == "__main__":
    image_saver = ImageSaver()
    image_saver.run()