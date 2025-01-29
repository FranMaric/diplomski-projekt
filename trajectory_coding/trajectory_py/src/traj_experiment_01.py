#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, PoseStamped, PoseArray
from std_msgs.msg import String


def create_point(x, y, z, time_from_start_secs):
    """Creates a MultiDOFJointTrajectoryPoint."""
    transform = Transform()
    transform.translation.x = x
    transform.translation.y = y
    transform.translation.z = z
    transform.rotation.x = 0.0
    transform.rotation.y = 0.0
    transform.rotation.z = 0.0
    transform.rotation.w = 1.0  # Identity quaternion

    if time_from_start_secs < 4:
        velocities = Twist()  # Zero velocity
        velocities.linear = Vector3(2, 2, 2)
        velocities.angular = Vector3(0, 0, 1)

        accelerations = Twist()  # Zero acceleration
        accelerations.linear = Vector3(2, 2, 2)
        accelerations.angular = Vector3(0, 0, 0)
    else: 
        velocities = Twist()  # Zero velocity
        velocities.linear = Vector3(10, 10, 10)
        velocities.angular = Vector3(0, 0, 1)

        accelerations = Twist()  # Zero acceleration
        accelerations.linear = Vector3(5, 5, 5)
        accelerations.angular = Vector3(0, 0, 0)

    time_from_start = rospy.Duration(time_from_start_secs)

    return MultiDOFJointTrajectoryPoint([transform], [velocities], [accelerations], time_from_start)


class TrajectoryPublisherNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('traj_experiment_01', anonymous=True)

        # Publisher for the trajectory points
        self.pub = rospy.Publisher('/duckorange/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=10)
        self.pub_triger = rospy.Publisher('/triger', PoseStamped, queue_size=10)

        # Subscriber to the trigger topic
        rospy.Subscriber('/drone/landing_pose', PoseStamped, self.trigger_callback)
        rospy.Subscriber('/fastp_trajectory', PoseArray, self.fastp_trajectory_callback)

        rospy.loginfo("TrajectoryPublisherNode is ready and waiting for triggers.")

    def trigger_callback(self, msg):
        """Callback function for the trigger topic."""
        rospy.loginfo("Received trigger: %s", msg)

        self.publish_pose(msg)

    def fastp_trajectory_callback(self, msg):
        rospy.loginfo("Received PoseArray with %d poses", len(msg.poses))

        trajectory_msg = MultiDOFJointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.header.frame_id = 'world'
        trajectory_msg.joint_names = ['base_link']

        points = []
        time_from_start_secs = 0.0  # Increment time for each point
        time_step = 1.0  # Example time step between points

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            points.append(create_point(x, y, z, time_from_start_secs))
            time_from_start_secs += time_step

        trajectory_msg.points = points

        rospy.loginfo("Publishing MultiDOFJointTrajectory with points")
        self.pub.publish(trajectory_msg)

    def publish_trajectory(self):
        """Creates and publishes the trajectory message with 3 points."""
        trajectory_msg = MultiDOFJointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.header.frame_id = 'world'
        trajectory_msg.joint_names = ['base_link']

        # Add trajectory points
        points = [
            create_point(4, 4, 4, 1),
            create_point(8, 4, 8, 2),
            create_point(10, 4, 10, 3),
            create_point(4, 4, 12, 4)
        ]

        trajectory_msg.points = points

        # Publish the trajectory
        rospy.loginfo("Publishing trajectory with 3 points.")
        self.pub.publish(trajectory_msg)

    def publish_pose(self, goal_pose): 
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'world'  

        # Set position from goal_pose
        pose_msg.pose.position.x = goal_pose.pose.position.x
        pose_msg.pose.position.y = goal_pose.pose.position.y
        pose_msg.pose.position.z = goal_pose.pose.position.z

        # Keep orientation fixed
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        rospy.loginfo(f"Publishing PoseStamped: position=({pose_msg.pose.position.x}, {pose_msg.pose.position.y}, {pose_msg.pose.position.z})")
        self.pub_triger.publish(pose_msg)

    def spin(self):
        """Keeps the node running."""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = TrajectoryPublisherNode()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down TrajectoryPublisherNode.")
