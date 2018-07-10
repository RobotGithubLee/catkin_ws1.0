#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>



#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <unistd.h>

#include "bernstin.h"

#define pi 3.1415926

using namespace std;


float amcl_pose[3]={0,0,0};
float last_pose_x=0;
float last_pose_y=0;

void amclposCallback(const geometry_msgs::PoseWithCovarianceStamped &amclPose)
{
  double roll,pitch,yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(amclPose.pose.pose.orientation,quat);
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

  last_pose_x=amcl_pose[0];
  last_pose_y=amcl_pose[1];

  amcl_pose[0]=amclPose.pose.pose.position.x;
  amcl_pose[1]=amclPose.pose.pose.position.y;
  amcl_pose[2]=yaw;

  //ROS_INFO("amcl_x=%.2f y=%.2f yaw=%.2f",amcl_pose[0],amcl_pose[1],amcl_pose[2]);
}



float normalize(float z)
{
  return atan2(sin(z),cos(z));
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


float distanceOfPoint2Point(float x1, float y1, float x2, float y2)
{
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));  
}




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "planner");

  ros::NodeHandle n;
  
  fstream inStream;

  ofstream outStream;

  float delta_pose[3];

  float goals[10240][3];
  

  float angel_flag=0;
  float v_flag=0;

  float u=1,beta=0.5;
  float d_Now2Amcl=0;
  float Px[4],Py[4];
  float Pc[2];
  float K;

  bool v_f=true;

  int i=0,count;
  
  nav_msgs::Path path;
  nav_msgs::Path Bezierpath;
  

  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::PoseStamped pose_goal;

  ros::Subscriber sub = n.subscribe("amcl_pose", 100, amclposCallback);
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmdVelocity", 100);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path",1);
  ros::Publisher Bezierpath_pub = n.advertise<nav_msgs::Path>("Bezierpath",1);
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal",1);


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
   //ROS_INFO("goals_x=%.3f y=%.3f yaw=%.3f",goals[count][0],goals[count][1],goals[count][2]);

 
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
  path.poses.pop_back();
  ROS_INFO("count=%d",count--);
  inStream.close();
  




  ros::Rate loop_rate(10);
  while(ros::ok())
  {
   current_time = ros::Time::now();
  
   pose_goal= path.poses[i];
   pose_goal.header.stamp=current_time;
   goal_pub.publish(pose_goal);
   path_pub.publish(path);


   if(i>=count)
   {
     cmdVel.linear.x=0;
     cmdVel.angular.z=0;
     cmd_pub.publish(cmdVel);
   }
   else
   {
     d_Now2Amcl=distanceOfPoint2Point(goals[i][0],goals[i][1],amcl_pose[0],amcl_pose[1]);
     angel_flag=(goals[i][0]-amcl_pose[0])*(amcl_pose[0]-last_pose_x)+(goals[i][1]-amcl_pose[1])*(amcl_pose[1]-last_pose_y);
    
    
    if(d_Now2Amcl<0.02||angel_flag<0 ||u>=1)
    {
      ROS_INFO("d_Now2Amcl=%.2f,angel_flag=%.2f,u=%.2f",d_Now2Amcl,angel_flag,u);  
      i+=10;
      u=0;
      d_Now2Amcl=distanceOfPoint2Point(goals[i][0],goals[i][1],amcl_pose[0],amcl_pose[1]);

      Px[0]=amcl_pose[0];
      Py[0]=amcl_pose[1];
 
      Px[1]=amcl_pose[0]+beta*d_Now2Amcl*cos(normalize(amcl_pose[2]));
      Py[1]=amcl_pose[1]+beta*d_Now2Amcl*sin(normalize(amcl_pose[2]));
  
      Px[2]=goals[i][0]-beta*d_Now2Amcl*cos(normalize(goals[i][2]));
      Py[2]=goals[i][1]-beta*d_Now2Amcl*sin(normalize(goals[i][2]));

      Px[3]=goals[i][0];
      Py[3]=goals[i][1];
      
      float angle_=atan2((Py[3]-Py[0]),(Px[3]-Px[0]));
      if(normalize(angle_-amcl_pose[2])>0)
        v_flag=1;
      else
        v_flag=-1;
      
     if((angle_-atan2((Py[1]-Py[0]),(Px[1]-Px[0])))*(angle_-atan2((Py[2]-Py[0]),(Px[2]-Px[0])))<0)
       v_f=true;
     else
       v_f=false;
      
      geometry_msgs::PoseStamped pose_Bezier;
      
      Bezierpath.header.stamp=current_time;
      Bezierpath.header.frame_id="map";
      Bezierpath.poses.clear();
      ROS_INFO("%.2f,%.2f,%.2f,%.2f",Px[0],Px[1],Px[2],Px[3]);  
      ROS_INFO("%.2f,%.2f,%.2f,%.2f",Py[0],Py[1],Py[2],Py[3]);  
     while(u<=1)
     {
      PointOnBezierCurve(3,u,Px,Py,Pc);
      pose_Bezier.pose.position.x = Pc[0];
      pose_Bezier.pose.position.y = Pc[1];
      pose_Bezier.header.stamp=current_time;
      pose_Bezier.header.frame_id="map";
      Bezierpath.poses.push_back(pose_Bezier);
      u+=0.01;
     }   
     
     ROS_INFO("i=%d",i);
     u=0.0;
    }
     Bezierpath_pub.publish(Bezierpath);
       
     PointOnBezierCurve(3,u,Px,Py,Pc);
     angel_flag=(Pc[0]-amcl_pose[0])*(amcl_pose[0]-last_pose_x)+(Pc[1]-amcl_pose[1])*(amcl_pose[1]-last_pose_y);
       //float deta_d=distanceOfPoint2Point(Pc[i][0],Pc[i][1],amcl_pose[0],amcl_pose[1]);
     while(angel_flag<0)
     {
       u+=0.01; 
       PointOnBezierCurve(3,u,Px,Py,Pc);
       angel_flag=(Pc[0]-amcl_pose[0])*(amcl_pose[0]-last_pose_x)+(Pc[1]-amcl_pose[1])*(amcl_pose[1]-last_pose_y);  
       if(u>=1)
         break; 
     } 
      if(u>0.5 &&v_f)
      {
        v_flag*=-1;
        v_f=false;
      }   
       K=v_flag*CurvatureOnBezierCeuve(3,u,Px,Py); 
       cmdVel.linear.x=0.5; //set vel
       float w=cmdVel.linear.x*K;
       if(w>0.15)
       {
         w=0.15;
         cmdVel.linear.x=w/K;
       }
       cmdVel.angular.z=w;
     }
     cmd_pub.publish(cmdVel);
     ROS_INFO("i=%d u=%.2f v=%.2f w=%.3f K=%.2f",i,u,cmdVel.linear.x,cmdVel.angular.z,K);
    
    
   ros::spinOnce();
   loop_rate.sleep();

  }

  outStream.close();
  return 0;
}
