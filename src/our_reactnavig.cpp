#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double obstacle_distance;

double getTurnAngularVelocity() {
    // Set a fixed angular velocity for turning
    // Positive value for turning left, negative value for turning right
    static double turn_velocity = 0.25; // Adjust this value to control the turning speed
    return turn_velocity;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    obstacle_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "our_reactnavig");
    ros::NodeHandle n;

    // Publisher for /cmd_vel
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    // Subscriber for /base_scan
    ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laserCallback);

    ros::Rate loop_rate(10); // 10 Hz

    // Initializations:
    geometry_msgs::Twist cmd_vel_msg;

    while (ros::ok()) {
        if (obstacle_distance < 0.8) {
            // When an obstacle is detected, stop and set a larger angular velocity for turning
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = getTurnAngularVelocity(); // Set fixed angular velocity for turning
        } else {
            // When no obstacle, move forward with an increased linear velocity
            cmd_vel_msg.linear.x = 0.8; // Increase this value for faster linear velocity
            cmd_vel_msg.angular.z = 0.0;
        }

        // Publish velocity commands:
        cmd_vel_pub.publish(cmd_vel_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

