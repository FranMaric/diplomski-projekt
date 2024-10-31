#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <cmath>

class TrajectoryPublisherNode {
public:
    TrajectoryPublisherNode() {
        ros::NodeHandle nh;

        trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/red/tracker/input_trajectory", 10);

        start_sub_ = nh.subscribe("/start_trajectory", 10, &TrajectoryPublisherNode::startTrajectoryCallback, this);

        ROS_INFO("Trajectory Publisher Node Initialized. Waiting for /start_trajectory to publish trajectory.");
    }

    void publishTrajectory() {
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        int num_points = 10;
        double radius = 5.0;

        for (int i = 0; i < num_points; ++i) {
            double t = i * (2 * M_PI / num_points);
            double x = radius * std::sin(t);
            double y = radius * std::sin(t) * std::cos(t);
            double z = 2.0;

            geometry_msgs::Transform transform;
            transform.translation.x = x;
            transform.translation.y = y;
            transform.translation.z = z;
            transform.rotation.w = 1.0;

            geometry_msgs::Twist velocities;
            velocities.linear.x = 5.0;
            velocities.linear.y = 5.0;
            velocities.linear.z = 0.0;

            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            point.transforms.push_back(transform);
            point.velocities.push_back(velocities);

            trajectory_msg.points.push_back(point);
        }

        trajectory_msg.joint_names.push_back("uav_joint");

        ROS_INFO("Publishing square trajectory!!");
        trajectory_pub_.publish(trajectory_msg);
    }

private:
    ros::Publisher trajectory_pub_;
    ros::Subscriber start_sub_;

    void startTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg) {
        ROS_INFO("Received message on /start_trajectory, starting trajectory publishing.");
        publishTrajectory();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher_node");
    TrajectoryPublisherNode node;
    ros::spin();
    return 0;
}
