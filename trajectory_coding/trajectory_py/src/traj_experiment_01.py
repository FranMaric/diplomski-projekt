#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, PoseStamped, PoseArray, Pose


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
        rospy.init_node('traj_experiment_01', anonymous=True)

        self.pub = rospy.Publisher('/duckorange/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=10)
        self.pub_triger = rospy.Publisher('/drone0/planning/triger/array', PoseArray, queue_size=10)

        rospy.Subscriber('/drone/landing_pose', PoseStamped, self.trigger_callback)
        rospy.Subscriber('/fastp_trajectory', PoseArray, self.fastp_trajectory_callback)
        rospy.Subscriber('/duckorange/pose', PoseStamped, self.pose_callback)

        self.drone_pose = Pose()

        rospy.loginfo("TrajectoryPublisherNode is ready and waiting for triggers.")

    def trigger_callback(self, msg):
        """Callback function for the trigger topic."""

        #self.publish_trajectory()
        rospy.loginfo("Received trigger: %s", msg.pose.position)
        self.publish_pose(msg)

    def pose_callback(self, msg):
        self.drone_pose.position.x = msg.pose.position.x
        self.drone_pose.position.y = msg.pose.position.y
        self.drone_pose.position.z = msg.pose.position.z

        self.drone_pose.orientation.x = msg.pose.orientation.x
        self.drone_pose.orientation.y = msg.pose.orientation.y
        self.drone_pose.orientation.z = msg.pose.orientation.z
        self.drone_pose.orientation.w = msg.pose.orientation.w
    
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
            x = -1*pose.position.y 
            y = pose.position.x
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

        points = [
            create_point(4, 4, 4, 1),
            create_point(8, 4, 8, 2),
            create_point(10, 4, 10, 3),
            create_point(4, 4, 12, 4)
        ]

        trajectory_msg.points = points

        rospy.loginfo("Publishing trajectory with 3 points.")
        self.pub.publish(trajectory_msg)

    def publish_pose(self, target_pose): 
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "world" 

        target = Pose()
        target.position.x = target_pose.pose.position.x
        target.position.y = target_pose.pose.position.y
        target.position.z = target_pose.pose.position.z
        target.orientation = target_pose.pose.orientation

        pose_array.poses.append(target)
        pose_array.poses.append(self.drone_pose)

        rospy.loginfo(f"Publishing array")
        self.pub_triger.publish(pose_array)

    def spin(self):
        """Keeps the node running."""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TrajectoryPublisherNode()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down TrajectoryPublisherNode.")

