#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double obstacle_distance;
bool obstacle_detected = false;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  // Find the minimum distance to the obstacle
  obstacle_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
  
  // Check if any obstacle is too close (e.g., within 0.5 meters)
  if (obstacle_distance < 0.5) {
    obstacle_detected = true;
  } else {
    obstacle_detected = false;
  }
}

int main(int argc, char **argv){
    
  ros::init(argc, argv, "our_reactnavig");
  
  ros::NodeHandle n;

  // Publisher for /cmd_vel
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  // Subscriber for /base_scan
  ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laserCallback);

  ros::Rate loop_rate(10); // 10 Hz

  // Initializations
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0.5;   // Set initial linear velocity
  cmd_vel_msg.angular.z = 0.0;  // Set initial angular velocity

  while (ros::ok()){
    
    if (obstacle_detected) {
      // If an obstacle is detected, turn away from it
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.3; // Adjust the angular velocity as needed
    } else {
      // If no obstacle is detected, move forward
      cmd_vel_msg.linear.x = 0.5; // Adjust the linear velocity as needed
      cmd_vel_msg.angular.z = 0.0;
    }

    // Publish velocity commands
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
} 

