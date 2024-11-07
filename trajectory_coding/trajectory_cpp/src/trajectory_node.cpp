#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <my_custom_msgs/Point6DOF.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  
#include <cmath>

class TrajectoryPublisherNode {
public:
    TrajectoryPublisherNode() {
        ros::NodeHandle nh;

        trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/duckorange/tracker/input_trajectory", 10);

        start_sub_ = nh.subscribe("/start_trajectory", 10, &TrajectoryPublisherNode::startTrajectoryCallback, this);
        position_sub_ = nh.subscribe("/duckorange/pose", 10, &TrajectoryPublisherNode::positionCallback, this);

        ROS_INFO("Trajectory Publisher Node Initialized. Waiting for /start_trajectory to publish trajectory.");
    }

    void publishTrajectory(const geometry_msgs::PoseStamped& start_point, const geometry_msgs::PoseStamped& end_point) {
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

        int num_points = 3;

        ROS_INFO("Starting Point: x=%.2f, y=%.2f, z=%.2f",
                start_point.pose.position.x, start_point.pose.position.y, start_point.pose.position.z);
        ROS_INFO("Target Point: x=%.2f, y=%.2f, z=%.2f, orientation (w=%.2f, x=%.2f, y=%.2f, z=%.2f)",
                end_point.pose.position.x, end_point.pose.position.y, end_point.pose.position.z,
                end_point.pose.orientation.w, end_point.pose.orientation.x, end_point.pose.orientation.y, end_point.pose.orientation.z);

        double dx = end_point.pose.position.x - start_point.pose.position.x;
        double dy = end_point.pose.position.y - start_point.pose.position.y;
        double dz = end_point.pose.position.z - start_point.pose.position.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        dx /= distance;
        dy /= distance;
        dz /= distance;

        for (int i = 0; i < num_points; ++i) {
            double t = (i * distance) / (num_points - 1);
            double x = start_point.pose.position.x + t * dx;
            double y = start_point.pose.position.y + t * dy;
            double z = start_point.pose.position.z + t * dz;

            geometry_msgs::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;

            geometry_msgs::Transform transform;
            transform.translation.x = pose.position.x;
            transform.translation.y = pose.position.y;
            transform.translation.z = pose.position.z;

            if (i < num_points - 1) {
                transform.rotation.w = 1.0;
                transform.rotation.x = 0.0;
                transform.rotation.y = 0.0;
                transform.rotation.z = 0.0;
            } else {
                transform.rotation = end_point.pose.orientation;
            }

            geometry_msgs::Twist velocities;
            if (i < num_points - 1) {
                velocities.linear.x = 2.0;
                velocities.linear.y = 2.0;
                velocities.linear.z = 0.0;
            } else {
                velocities.linear.x = 5.0;
                velocities.linear.y = 5.0;
                velocities.linear.z = 0.0;
            }

            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            point.transforms.push_back(transform);
            point.velocities.push_back(velocities);
            trajectory_msg.points.push_back(point);

            ROS_INFO("Waypoint %d: x=%.2f, y=%.2f, z=%.2f, orientation (w=%.2f, x=%.2f, y=%.2f, z=%.2f)",
                    i, transform.translation.x, transform.translation.y, transform.translation.z,
                    transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
        }

        trajectory_msg.joint_names.push_back("uav_joint");

        ROS_INFO("Publishing trajectory with %d points.", num_points);
        trajectory_pub_.publish(trajectory_msg);
    }


private:
    ros::Publisher trajectory_pub_;
    ros::Subscriber start_sub_;
    ros::Subscriber position_sub_;
    geometry_msgs::PoseStamped duckorange_position_;
    bool duckorange_position_set_;

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        duckorange_position_ = *msg;
        duckorange_position_set_ = true;
    }

    void startTrajectoryCallback(const my_custom_msgs::Point6DOF::ConstPtr& msg) {
        if (!duckorange_position_set_) {
            ROS_WARN("Duckorange position not set yet. Waiting for /duckorange/position data.");
            return;
        }

        ROS_INFO("Received message on /start_trajectory, starting trajectory publishing.");

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "duckorange_camera";  
        target_pose.header.stamp = ros::Time(0);            
        target_pose.pose.position.x = msg->x;
        target_pose.pose.position.y = msg->y;
        target_pose.pose.position.z = msg->z;

        tf2::Quaternion q;
        q.setRPY(msg->roll, msg->pitch, msg->yaw);
        target_pose.pose.orientation = tf2::toMsg(q);  

        // geometry_msgs::PoseStamped transformed_pose;
        // try {
        //     tf_buffer_.transform(target_pose, transformed_pose, "world", ros::Duration(1.0));
        //     ROS_INFO("Transformed target pose to /world frame: position x=%.2f, y=%.2f, z=%.2f, orientation x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z,
        //             transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, 
        //             transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w);
        // } catch (tf2::TransformException& ex) {
        //     ROS_WARN("Could not transform target pose to /world frame: %s", ex.what());
        //     return;
        // }
        publishTrajectory(duckorange_position_, target_pose);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher_node");
    TrajectoryPublisherNode node;
    ros::spin();
    return 0;
}
