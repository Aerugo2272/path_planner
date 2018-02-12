#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

geometry_msgs::Pose2D current_pose;
ros::Publisher pub_pose2D;

void odomCallback(const nav_msgs::OdometryConstPtr& msg){
  //current linear position
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;
  
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_pose.theta = yaw;
}

int main(int argc, char** argv){
  
  const double PI = 3.14159265358979323846;
  
  ros::init(argc, argv, "path_planner");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",10);

  //publisher for current positon
  ros::Rate rate(30);
  geometry_msgs::Twist move;
  float goal_x, goal_y;
  float error = 0.15;
  bool goal_reached = false;
  std::cout<<"Enter initial goal: ";
  std::cin >> goal_x >> goal_y;

  while(ros::ok()){
    float df_x = goal_x - current_pose.x;
    float df_y = goal_y - current_pose.y;
    float angle_to_goal = atan2(df_y,df_x);
    float df_theta = angle_to_goal - current_pose.theta;
    if(df_theta>PI){
      df_theta = df_theta - 2*PI;
    }
    else if(df_theta<-PI){
      df_theta = df_theta + 2*PI;
    }
    
    ROS_INFO("d_theta %f", df_theta);
    ROS_INFO("dx %f", df_x);
    ROS_INFO("dy %f", df_y);
    ROS_INFO("angle_to_goal %f", angle_to_goal);
    
    if(fabs(df_theta) < error && fabs(df_x)>error || fabs(df_y)>error){
      move.linear.x = 0.25;
      move.angular.z = 0;
      movement_pub.publish(move);
      ROS_INFO("Move Straight");  
    }

    
    else if(df_theta > error){
      move.linear.x = 0;
      move.angular.z = 0.4;
      movement_pub.publish(move);
      ROS_INFO("Turn Anticlockwise");  
    }
    
    else if(df_theta < 0-error){
      move.linear.x = 0;
      move.angular.z = -0.4;
      movement_pub.publish(move);
      ROS_INFO("Turn Clockwise");  
    }
    

    else if(fabs(df_x) < error && fabs(df_y) < error){
      move.linear.x = 0;
      move.angular.z = 0;
      movement_pub.publish(move);
      ROS_INFO("Stop");  
    }
  
    ros::spinOnce();
    rate.sleep();
    
    
  }
  return 0;
}

