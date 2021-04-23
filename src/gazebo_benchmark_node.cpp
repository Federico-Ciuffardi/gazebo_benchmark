/*
 * To run gmapping you should start gmapping:
 * rosrun gmapping slam_gmapping scan:=lidar_topic _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30
 * _delta:=0.2
 */

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/init.h"
#include "ros/ros.h"
#include <ros/console.h>

#include <utility>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <math.h>
#include <stdexcept>      // std::out_of_range
#include <vector>         // std::vector

#define TIME_STEP 32
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9
#define NMOTORS 2

using namespace std;

ros::NodeHandle *n;

enum State { FORWARD , BACKWARD };
int state = FORWARD;

ros::Publisher cmdVelPub;

// motors
void updateSpeed(float speed) {
  geometry_msgs::Vector3 linear;
  linear.x = speed;
  linear.y = 0;
  linear.z = 0;

  geometry_msgs::Vector3 angular;
  angular.x = 0;
  angular.y = 0;
  angular.z = 0;

  geometry_msgs::Twist vel;
  vel.linear=linear;
  vel.angular=angular;

  cmdVelPub.publish(vel);
}

// lidar
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  unsigned long  targets[2] = {scan->ranges.size()/2, 0};

  unsigned long target_laser_index = targets[state]; 

  float distance;
  try {
    distance = scan->ranges.at(target_laser_index);
  } catch (const out_of_range& oor){
    ROS_ERROR_STREAM("Out of Range error: " << oor.what());
    return;
    ros::shutdown();
    exit(1);
  }

  bool tooClose = scan->ranges[target_laser_index] < 1;

  state = (state + tooClose ) % 2;

  float speed = min(scan->ranges[targets[BACKWARD]], scan->ranges[targets[FORWARD]])/5;
  if ( state == BACKWARD ){
    speed = -speed;
  }
  updateSpeed(speed);
}

// at SIGINT quit
void quit(int sig) {
  ROS_INFO("User stopped the node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  std::string controllerName;
  ros::init(argc, argv, "benchmark_controller");
  ros::NodeHandle n;
  ros::Duration(5).sleep(); // first ROS_INFO_STREAM did not show on rqt_console and
                            // also webots services failed otherwise 

  string ns = ros::this_node::getNamespace();
  ROS_INFO_STREAM("ns = " << ns);

  signal(SIGINT, quit);

  cmdVelPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Subscriber sub = n.subscribe("laser/scan", 1000, lidarCallback);
  // main loop
  ROS_INFO("Initialized!");

  ros::spin();
  ros::shutdown();
  return 0;
}
