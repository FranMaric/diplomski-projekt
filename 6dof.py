import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

class PoseEstimationNode:
    def __init__(self, object_points):
        rospy.init_node('pose_estimation_node')

        # Set object points from the input parameter
        self.object_points = np.array(object_points, dtype=np.float32)

        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_pose = None  # Store the last estimated pose

        # ROS subscribers and publisher
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher("/object_pose", PoseStamped, queue_size=10)

        self.bridge = CvBridge()

        # Timer to publish pose at a fixed frequency
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_pose)  # Publish every 0.1 seconds (10 Hz)

    def camera_info_callback(self, camera_info):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(camera_info.K).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info.D)

    def image_callback(self, msg, image_points):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera intrinsics not yet received.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8") 

            # Convert image_points from list to numpy array
            image_points = np.array(image_points, dtype=np.float32)

            # Visualize detected points on the image
            for point in image_points:
                cv2.circle(frame, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)  # zaokruzi dani koordinate

            cv2.imshow("Detected Points", frame)  
            cv2.waitKey(1)  

            if len(image_points) >= 4:
                success, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs)
                if success:
                    self.last_pose = (rvec, tvec)  

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
    def publish_pose(self, event):
        if self.last_pose is not None:
            rvec, tvec = self.last_pose

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Prepare PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_frame"

            # Set translation (position)
            pose_msg.pose.position.x = tvec[0]
            pose_msg.pose.position.y = tvec[1]
            pose_msg.pose.position.z = tvec[2]

            # Convert rotation matrix to quaternion
            quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            # Publish pose
            self.pose_pub.publish(pose_msg)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        q = np.zeros(4)
        q[3] = np.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
        q[0] = np.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
        q[1] = np.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
        q[2] = np.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
        q[0] *= np.sign(q[0] * (R[2, 1] - R[1, 2]))
        q[1] *= np.sign(q[1] * (R[0, 2] - R[2, 0]))
        q[2] *= np.sign(q[2] * (R[1, 0] - R[0, 1]))
        return q


if __name__ == '__main__':
    try:
        # Example of object points to be passed when creating the node
        object_points = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ]
        
        node = PoseEstimationNode(object_points)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass