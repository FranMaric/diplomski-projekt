#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from math import pi
import tf.transformations as tf


current_image = None

def image_callback(data):
    global current_image

    current_image = data

bridge = CvBridge()

def save_jpg_image(index):
    if current_image == None:
        print('current_image is None. Try again later')
        return
    
    try:
        cv2_img = bridge.imgmsg_to_cv2(current_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        img_name = 'images/camera_image_' + str(index) + '.jpeg'
        cv2.imwrite(img_name, cv2_img)
    rospy.loginfo('Saved image as ' + os.getcwd() + '/' + img_name)


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


if __name__ == '__main__':
    rospy.Subscriber('/duckorange/camera/color/image_raw', Image, image_callback)

    rospy.sleep(2)

    set_waypoint(2, 3, 3, pi)

    rospy.sleep(5)

    save_jpg_image(0)

    set_waypoint(2, 0, 4, 2*pi/3)

    rospy.sleep(5)

    save_jpg_image(1)

    set_waypoint(3, -3, 2, pi)

    rospy.sleep(5)

    save_jpg_image(2)

    set_waypoint(0, -3, 1, 0)

    rospy.sleep(5)

    set_waypoint(-4, -3, 3.5, pi/4)

    rospy.sleep(5)

    save_jpg_image(3)

    set_waypoint(-5, 0, 5, 0)

    rospy.sleep(5)

    save_jpg_image(4)

    set_waypoint(-5, 2.5, 5, 0)

    rospy.sleep(5)

    save_jpg_image(5)
