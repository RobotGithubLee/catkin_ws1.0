#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>


#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>


#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <unistd.h>

#define pi 3.1415926

using namespace std;


float amcl_pose[3];


void amclposCallback(const geometry_msgs::PoseWithCovarianceStamped &amclPose)
{
  double roll,pitch,yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(amclPose.pose.pose.orientation,quat);
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  amcl_pose[0]=amclPose.pose.pose.position.x;
  amcl_pose[1]=amclPose.pose.pose.position.y;
  amcl_pose[2]=yaw;

  ROS_INFO("amcl_x=%.2f y=%.2f yaw=%.2f",amcl_pose[0],amcl_pose[1],amcl_pose[2]);
}

float angle_diff(float a,float b)
{
  float d;
  d=a-b;
  if(d>pi)
    d=d-2*pi;
  else if(-1*d>pi)
    d=d+2*pi;
  return (d);
}

float  PointToSegDist(float x, float y, float x1, float y1, float x2, float y2)
{

   float cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
   float d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
   float r = cross / d2;
   float px = x1 + (x2 - x1) * r;
   float py = y1 + (y2 - y1) * r;
   return sqrt((x - px) * (x - px) + (py - y1) * (py - y1));
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "planner");

  ros::NodeHandle n;
  
  fstream inStream;

  ofstream outStream;

  float delta_pose[3];

  float goals[1024][3];
  
  
  float last_pose_x=0;
  float last_pose_y=0;

  int i=1,count;
  
  nav_msgs::Path path;

  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::PoseStamped pose_goal;

  ros::Time current_time;

   geometry_msgs::Twist cmdVel;

  path.header.stamp=current_time;
  path.header.frame_id="map";

  outStream.open("publishVel.txt",std::ios::out);

  inStream.open("goal.txt",std::ios::in);


  
  if(!inStream.is_open())
  {
    ROS_INFO("open goal.txt false!!!");  
    return -1;     
  }
  while(!inStream.eof())
  {
   current_time = ros::Time::now();
   inStream>>goals[count][0]>>goals[count][1]>>goals[count][2];
   ROS_INFO("goals_x=%.3f y=%.3f yaw=%.3f",goals[count][0],goals[count][1],goals[count][2]);

 
   pose_stamped.pose.position.x = goals[count][0];
   pose_stamped.pose.position.y = goals[count][1];

   geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(goals[count][2]);
   pose_stamped.pose.orientation.x = goal_quat.x;
   pose_stamped.pose.orientation.y = goal_quat.y;
   pose_stamped.pose.orientation.z = goal_quat.z;
   pose_stamped.pose.orientation.w = goal_quat.w;
  
   pose_stamped.header.stamp=current_time;
   pose_stamped.header.frame_id="map";
   path.poses.push_back(pose_stamped);
   
   count++;
  }
  ROS_INFO("count=%d",count--);
  path.poses.pop_back();
  inStream.close();
  

  ros::Subscriber sub = n.subscribe("amcl_pose", 100, amclposCallback);
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmdVelocity", 100);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path",1);
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal",1);




  ros::Rate loop_rate(10);
  while(n.ok())
  {
   current_time = ros::Time::now();

   pose_goal= path.poses[i];
   pose_goal.header.stamp=current_time;  
   goal_pub.publish(pose_goal);



   delta_pose[0]=goals[i][0]-amcl_pose[0];
   delta_pose[1]=goals[i][1]-amcl_pose[1];
   delta_pose[2]=goals[i][2]-amcl_pose[2];
   float delta_d=sqrt(delta_pose[0]*delta_pose[0]+delta_pose[1]*delta_pose[1]);
   float angel_flag=delta_pose[0]*(amcl_pose[0]-last_pose_x)+delta_pose[1]*(amcl_pose[1]-last_pose_y);
   if(i>=count)
   {
     cmdVel.linear.x=0;
     cmdVel.angular.z=0;
     cmd_pub.publish(cmdVel);
   }
   else
   {
    float angle_line2x=atan2(delta_pose[1],delta_pose[0]);
    float angle_amclpos2line=angle_diff(amcl_pose[2],angle_line2x);
    float angle_goalpos2line=angle_diff(goals[i][2],angle_line2x);



    float diff_angle=atan2(sin(delta_pose[2]),cos(delta_pose[2]));
    if(angle_amclpos2line*angle_goalpos2line>0 &&i>1)
    {
     float dist=PointToSegDist(amcl_pose[1],amcl_pose[2],goals[i-1][0],goals[i-1][1],goals[i][0],goals[i][1]);
      if(dist>0.3||fabs(angle_amclpos2line)>0.2)
      {
       diff_angle=(-angle_amclpos2line);
       delta_d=delta_d/1.1;
       ROS_INFO("diff_angle=%.2f",diff_angle);
      }
     
   }

  cmdVel.linear.x=0.7; 
    cmdVel.angular.z=cmdVel.linear.x*sin(diff_angle)/(delta_d/2);

    float ww=fabs(cmdVel.angular.z);
    if(ww>0.15)
      ww=0.15;

    cmdVel.linear.x=fabs(ww-0.15)*2+0.2;
    if(cmdVel.linear.x>0.7)
       cmdVel.linear.x=0.7;
    else if(cmdVel.linear.x<0.2)
        cmdVel.linear.x=0.2;
   
   cmdVel.angular.z=cmdVel.linear.x*sin(diff_angle)/(delta_d/2);
   if(cmdVel.angular.z>0.15)
     cmdVel.angular.z=0.15;
   else if(cmdVel.angular.z<-0.15)
     cmdVel.angular.z=-0.15;

    outStream<<goals[i][0]<<"  "<<goals[i][1]<<"  "<<goals[i][2]<<"     ";
    outStream<<amcl_pose[0]<<"  "<<amcl_pose[1]<<"  "<<amcl_pose[2]<<"     ";
    outStream<<delta_pose[0]<<"  "<<delta_pose[1]<<"  "<<delta_pose[2]<<"     ";
    outStream<<delta_d<<"  "<<angel_flag<<"  "<<cmdVel.linear.x<<"  "<<cmdVel.angular.z<<"\n";
   }
   ROS_INFO("goals_x=%.2f y=%.2f yaw=%.2f  i=%d",goals[i][0],goals[i][1],goals[i][2],i);
   cmd_pub.publish(cmdVel);
  
   if(delta_d<0.2||angel_flag<0)
      i+=5;
   last_pose_x=amcl_pose[0];
   last_pose_y=amcl_pose[1];


  path_pub.publish(path);
  ros::spinOnce();
  loop_rate.sleep();

  }
  outStream.close();
  return 0;
}
