#include "ros/ros.h"
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#include <fstream>
#include <iostream>
#include <string>

using namespace std;


ofstream inStream;
nav_msgs::Path path;
float last_d=0,last_yaw=0;
void amclposCallback(const geometry_msgs::PoseWithCovarianceStamped &amcl_pose)
{
  geometry_msgs::PoseStamped pose_stamped;
  double roll,pitch,yaw,d,x,y;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(amcl_pose.pose.pose.orientation,quat);
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  x=amcl_pose.pose.pose.position.x;
  y=amcl_pose.pose.pose.position.y;
  d=sqrt(x*x+y*y);
  
  if(fabs(d-last_d)>0.4||fabs(yaw-last_yaw)>0.1)
 {
  inStream<<amcl_pose.pose.pose.position.x<<" "<<amcl_pose.pose.pose.position.y<<" "<<yaw<<"\n";
  last_d=d;
  last_yaw=yaw;
 }
  ROS_INFO("x=%.2f y=%.2f yaw=%.2f",x*1.0,y*1.0,yaw);
  

  pose_stamped.header.stamp=ros::Time::now();;
  pose_stamped.header.frame_id="map";
  pose_stamped.pose=amcl_pose.pose.pose;
  path.poses.push_back(pose_stamped);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Time current_time;
  inStream.open("goal.txt",std::ios::out);
  if(!inStream.is_open())
  {
    ROS_INFO("open goal.txt false!!!!");      
  }

  ros::Subscriber sub = n.subscribe("amcl_pose", 1000, amclposCallback);
  ros::Publisher  path_pub = n.advertise<nav_msgs::Path>("path",1);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
   current_time=ros::Time::now();
   //if(path.poses.size())
   path.header.stamp=current_time;
   path.header.frame_id="map";
   path_pub.publish(path);
   ros::spinOnce();              
  }
  inStream.close();
  return 0;
}
