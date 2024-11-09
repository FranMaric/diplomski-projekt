#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Transform, Twist
from my_custom_msgs.msg import Point6DOF
import math
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TrajectoryPublisherNode:
    def __init__(self):
        # Initialize the ROS Node
        rospy.init_node('trajectory_publisher_node', anonymous=True)

        # Publisher for trajectory
        self.trajectory_pub = rospy.Publisher('/duckorange/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=10)

        # Subscriber for start_trajectory and position updates
        self.start_sub = rospy.Subscriber('/start_trajectory', Point6DOF, self.start_trajectory_callback)
        self.position_sub = rospy.Subscriber('/duckorange/pose', PoseStamped, self.position_callback)

        # Initializations
        self.duckorange_position = None
        self.duckorange_position_set = False

        rospy.loginfo("Trajectory Publisher Node Initialized. Waiting for /start_trajectory to publish trajectory.")

    def position_callback(self, msg):
        # Callback to store the current position of Duckorange
        self.duckorange_position = msg
        self.duckorange_position_set = True

    def start_trajectory_callback(self, msg):#rostopic pub /start_trajectory my_package/Point6DOF "{x: 1.0, y: 2.0, z: 3.0, roll: 0.0, pitch: 0.5, yaw: 1.0}"

        if not self.duckorange_position_set:
            rospy.loginfo("Duckorange position not set yet. Waiting for /duckorange/pose data.")
            return

        rospy.loginfo("Received message on /start_trajectory, starting trajectory publishing.")

        # Create target PoseStamped message from the Point6DOF message
        target_pose = PoseStamped()
        target_pose.header.frame_id = "duckorange_camera"  
        target_pose.header.stamp = rospy.Time(0)            
        target_pose.pose.position.x = msg.x
        target_pose.pose.position.y = msg.y
        target_pose.pose.position.z = msg.z

        # Set orientation from roll, pitch, yaw
        q = quaternion_from_euler(msg.roll, msg.pitch, msg.yaw)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # Call the publishTrajectory function to publish the trajectory
        self.publish_trajectory(self.duckorange_position, target_pose)

    def publish_trajectory(self, start_point, end_point):
        trajectory_msg = MultiDOFJointTrajectory()

        num_points = 3

        # Log the start and end points
        rospy.loginfo("Starting Point: x=%.2f, y=%.2f, z=%.2f", start_point.pose.position.x, start_point.pose.position.y, start_point.pose.position.z)
        rospy.loginfo("Target Point: x=%.2f, y=%.2f, z=%.2f, orientation (w=%.2f, x=%.2f, y=%.2f, z=%.2f)",
                    end_point.pose.position.x, end_point.pose.position.y, end_point.pose.position.z,
                    end_point.pose.orientation.w, end_point.pose.orientation.x, end_point.pose.orientation.y, end_point.pose.orientation.z)
    
        # Extract starting and ending positions
        start_x, start_y, start_z = (
            start_point.pose.position.x,
            start_point.pose.position.y,
            start_point.pose.position.z
        )
        end_x, end_y, end_z = (
            end_point.pose.position.x,
            end_point.pose.position.y,
            end_point.pose.position.z
        )
        
        # Calculate step sizes for x, y, z
        step_x = (end_x - start_x) / (num_points + 1)
        step_y = (end_y - start_y) / (num_points + 1)
        step_z = (end_z - start_z) / (num_points + 1)
        
        # Generate intermediate PoseStamped points
        for i in range(1, num_points + 1):
            
            # Set positions
            transform = Transform()
            transform.translation.x = start_x + step_x * i
            transform.translation.y = start_y + step_y * i
            transform.translation.z = start_z + step_z * i

            if i < num_points - 1:
                transform.rotation.w = 1.0
                transform.rotation.x = 0.0
                transform.rotation.y = 0.0
                transform.rotation.z = 0.0
            else:
                transform.rotation = end_point.pose.orientation

            velocities = Twist()
            if i < num_points - 1:
                velocities.linear.x = 2.0
                velocities.linear.y = 2.0
                velocities.linear.z = 0.0
            else:
                velocities.linear.x = 5.0
                velocities.linear.y = 5.0
                velocities.linear.z = 0.0

            point = MultiDOFJointTrajectoryPoint()
            point.transforms.append(transform)
            point.velocities.append(velocities)
            trajectory_msg.points.append(point)

            # Log each intermediate pose (waypoint)
            rospy.loginfo("Waypoint %d: x=%.2f, y=%.2f, z=%.2f, orientation (w=%.2f, x=%.2f, y=%.2f, z=%.2f)",
                        i, transform.translation.x, transform.translation.y, transform.translation.z,
                        transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z)

        transform = Transform()
        transform.translation.x = end_point.pose.position.x
        transform.translation.y = end_point.pose.position.y
        transform.translation.z = end_point.pose.position.z
        # quaternion = tf.transformations.quaternion_from_euler(
        #         r,p,j)
        point = MultiDOFJointTrajectoryPoint()
        point.transforms.append(transform)
        trajectory_msg.points.append(point)
        rospy.loginfo("Waypoint %d: x=%.2f, y=%.2f, z=%.2f",
        i, transform.translation.x, transform.translation.y, transform.translation.z)
        trajectory_msg.joint_names.append("uav_joint")

        # Log final status
        rospy.loginfo("Publishing trajectory with %d points.", num_points)
        self.trajectory_pub.publish(trajectory_msg)

if __name__ == '__main__':
    # Start the node and spin to keep it running
    node = TrajectoryPublisherNode()
    rospy.spin()
