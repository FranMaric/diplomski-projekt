#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from math import pi
import tf.transformations as tf
import numpy as np

current_colored_image = None
current_depth_image = None

def color_image_callback(data):
    global current_colored_image

    current_colored_image = data


def depth_image_callback(data):
    global current_depth_image

    current_depth_image = data


bridge = CvBridge()

def save_color_image(index):
    if current_colored_image == None:
        rospy.logwarn('current_colored_image is None. Try again later')
        return
    
    try:
        cv2_img = bridge.imgmsg_to_cv2(current_colored_image, "bgr8")
        img_name = 'images/color_image_' + str(index) + '.jpeg'
        cv2.imwrite(img_name, cv2_img)
        rospy.loginfo('Saved image as ' + os.getcwd() + '/' + img_name)
    except CvBridgeError as e:
        rospy.logerr(e)
    

def save_depth_image(index):
    if current_depth_image == None:
        rospy.logwarn('current_depth_image is None. Try again later')
        return
    
    try:
        cv2_img = bridge.imgmsg_to_cv2(current_depth_image, desired_encoding="passthrough")

        if cv2_img.dtype != np.float32:
            rospy.logwarn(f"Expected 32-bit float depth image, but got a different of type {cv2_img.dtype}")
        
        npy_file_path = os.path.join("images", f"depth_image_distance_{index}.npy")
        np.save(npy_file_path, cv2_img)
        rospy.loginfo(f"32-bit depth image saved as NumPy array to {npy_file_path}")

        # Save a normalized version of the image for visualization
        normalized_depth = cv2.normalize(cv2_img, None, 0, 255, cv2.NORM_MINMAX)
        normalized_depth = normalized_depth.astype(np.uint8)  # Convert to 8-bit for saving as PNG
        png_file_path = os.path.join("images", f"depth_image_visualization_{index}.png")
        cv2.imwrite(png_file_path, normalized_depth)
        rospy.loginfo(f"Normalized depth image saved for visualization to {png_file_path}")
    except CvBridgeError as e:
        rospy.logerr(e)
    

def set_waypoint(x, y, z, yaw):
    pub = rospy.Publisher('/duckorange/tracker/input_pose', PoseStamped, queue_size=10)

    rospy.init_node('dipl_projekt', anonymous=True)

    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "base_link"

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    # Convert yaw to quaternion and set orientation
    quaternion = tf.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    
    pub.publish(goal)


def goto_and_take_photo(x, y, z, yaw, index):
    set_waypoint(x, y, z, yaw)

    rospy.sleep(5)

    save_color_image(index)
    save_depth_image(index)

if __name__ == '__main__':
    rospy.Subscriber('/duckorange/camera/color/image_raw', Image, color_image_callback)
    rospy.Subscriber('/duckorange/camera/depth/image_raw', Image, depth_image_callback)

    rospy.sleep(2)

    points = [
        [2, -6, 4, pi/2 + pi/8],
        [4, -5, 4, pi/2 + pi/4],
        [3, -1, 4, pi],
        [4, 2, 4, pi],
        [3, 5, 4, pi + pi/4],
        [1, 5, 4, pi + pi/4 + pi/10],
        [-4, 6, 4, -pi/2  + pi/4],
        [-6, 3, 4, -pi/8],
        [-6, 0, 4, 0],
        [-6, -3, 4, pi/8],
        [-5, -5, 4, pi/4],
    ]

    heights = [2, 4, 6]

    for z_index in range(len(heights)):
        for i in range(len(points)):
            x = points[i][0]
            y = points[i][1]
            z = heights[z_index]
            yaw = points[i][3]
            goto_and_take_photo(x, y, z, yaw, len(points) * z_index + i)
        set_waypoint(-1, -8, heights[z_index], 0)
        rospy.sleep(5)

