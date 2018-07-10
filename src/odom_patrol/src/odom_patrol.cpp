/*************************************************************************
    > File Name: odom_patrol.cpp
    > Author: Louis.Qiu
    > Mail: louis.qiu@cloudminds.com 
    > Created Time: Tue 17 Apr 2018 10:08:03 PM CST
 ************************************************************************/



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <turtle_actionlib/Velocity.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



float vx = 0.0;
float vy = 0.0;
float vth = 0.0;
float thta = 0.0;



void odomCallback(const geometry_msgs::Twist &twist)
{
  vx = twist.linear.x;
  vy = 0;
  vth = twist.angular.z;
  ROS_INFO("vx=%f vth=%f",vx,vth);
}



int main(int argc, char** argv){

  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber cmdvel_sub = n.subscribe("backVelocity", 1000,odomCallback);
  
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double lastth = 0.0;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10);
  while(n.ok())
  {

    ros::spinOnce();
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;

    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    ROS_INFO("x=%f y=%f thta=%f",x,y,th*180/3.14);
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[14] = 0.001;
    odom.pose.covariance[21] = 0.001;
    odom.pose.covariance[28] = 0.001;
    odom.pose.covariance[35] = 0.001;


    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    
    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[14] = 0.001;
    odom.twist.covariance[21] = 0.001;
    odom.twist.covariance[28] = 0.001;
    odom.twist.covariance[35] = 0.001;
    //send the odometry
    odom_pub.publish(odom);

    last_time = current_time;
    lastth=th;
    r.sleep();
  }
 return 0;
}


