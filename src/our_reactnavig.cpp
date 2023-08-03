#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <random> // Include the random library

double obstacle_distance;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
}

double getRandomAngularVelocity() {
    // Generate a random angular velocity between -1.0 and 1.0
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-3.0, 3.0);
    return dis(gen);
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

  while (ros::ok()){
    if (obstacle_distance < 0.8){
      // When an obstacle is detected, stop and set random angular velocity
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = getRandomAngularVelocity(); // Set random angular velocity
    } else {
      // When no obstacle, move forward with a constant linear velocity
      cmd_vel_msg.linear.x = 0.4;
      cmd_vel_msg.angular.z = 0.0; // Set random angular velocity
    }

   // Publish velocity commands:
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
