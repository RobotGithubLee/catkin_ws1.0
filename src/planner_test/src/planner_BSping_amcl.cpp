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

#include "BSping.h"

#define pi 3.1415926

using namespace std;


Point3D amcl_pose;
Point3D last_pose;


void amclposCallback(const geometry_msgs::PoseWithCovarianceStamped &amclPose)
{
  double roll,pitch,yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(amclPose.pose.pose.orientation,quat);
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

  last_pose.X=amcl_pose.X;
  last_pose.Y=amcl_pose.Y;

  amcl_pose.X=amclPose.pose.pose.position.x;
  amcl_pose.Y=amclPose.pose.pose.position.y;
  amcl_pose.Z=yaw;

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




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "planner");

  ros::NodeHandle n;
  
  fstream inStream;

  ofstream outStream;

  float delta_pose[3];

  Point3D Pc,Pc2;
  Point3D goals[10240];
  Point3D fitPoint[10240];
  float uk[10240];
  float U[10240];

  float angel_flag=0;
  float angle_plan;


  float u=0;
  float beta=0.2;
 

  int i=0,k=0,count;
  
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
   inStream>>goals[count].X>>goals[count].Y>>goals[count].Z;
 
   pose_stamped.pose.position.x = goals[count].X;
   pose_stamped.pose.position.y = goals[count].Y;
  
   pose_stamped.header.stamp=current_time;
   pose_stamped.header.frame_id="map";
   path.poses.push_back(pose_stamped);
   
   count++;
  }
  path.poses.pop_back();
  ROS_INFO("count=%d",count--);
  inStream.close();
  
  for(i=0;i<count;i+=5)
  {
    fitPoint[k].X=goals[i].X;
    fitPoint[k].Y=goals[i].Y;
    k++;
  }
  fitPoint[k].Y=goals[count-1].Y;
  count=k+1;
  ROS_INFO("fitPoint count=%d",count);
  getFitPointU(fitPoint,count,0,uk);
  getVectorU(count,uk,U);
  
  Bezierpath.header.stamp=current_time;
  Bezierpath.header.frame_id="map";
  Bezierpath.poses.clear();
  geometry_msgs::PoseStamped pose_Bezier;
  while(u<1-0.001)
  {
      Pc=getCurvePoint(u,count, U, fitPoint);
      pose_Bezier.pose.position.x = Pc.X;
      pose_Bezier.pose.position.y = Pc.Y;
      pose_Bezier.header.stamp=current_time;
      pose_Bezier.header.frame_id="map";
      Bezierpath.poses.push_back(pose_Bezier);
      u+=0.001;
     
     
  }   
  u=0;
  Pc.X=0;Pc.Y=0;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {

   path_pub.publish(path);
   Bezierpath_pub.publish(Bezierpath);

   current_time = ros::Time::now();
  
   pose_goal.pose.position.x = Pc.X;
   pose_goal.pose.position.y = Pc.Y;
   
   geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(Pc.Z);
   pose_goal.pose.orientation.x = goal_quat.x;
   pose_goal.pose.orientation.y = goal_quat.y;
   pose_goal.pose.orientation.z = goal_quat.z;
   pose_goal.pose.orientation.w = goal_quat.w;
   

   pose_goal.header.frame_id="map";
   pose_goal.header.stamp=current_time;
   goal_pub.publish(pose_goal);

   
   float d_amcl2goal=getDistance(amcl_pose,Pc);
   float angle_plan=atan2(Pc.Y-amcl_pose.Y,Pc.X-amcl_pose.X);
   float angle_amcl2plan=angle_diff(angle_plan,amcl_pose.Z);
   float angle_amcl2plan2=angle_diff(Pc.Z,amcl_pose.Z);
  
   float Vel_flag=cos(amcl_pose.Z)*(amcl_pose.X-last_pose.X)+sin(amcl_pose.Z)*(amcl_pose.Y-last_pose.Y);

   float angel_flag=(Pc.X-amcl_pose.X)*(amcl_pose.X-last_pose.X)+(Pc.Y-amcl_pose.Y)*(amcl_pose.Y-last_pose.Y);
   float angel_flag2=(Pc.X-amcl_pose.X)*(Pc2.X-Pc.X)+(Pc.Y-amcl_pose.Y)*(Pc2.Y-Pc.Y);
   
   ROS_INFO("angel_flag=%.4f,angel_flag2=%.4f d_amcl2goal=%.2f,Vel_flag=%f",angel_flag, angel_flag2,d_amcl2goal,Vel_flag);
   
   if(u>=1-0.0002)
   {
     cmdVel.linear.x=0;
     cmdVel.angular.z=0;
     cmd_pub.publish(cmdVel);
   }
   else
   {
    
    while((d_amcl2goal<1.0||angel_flag<0 || angel_flag2<0 || fabs(angle_amcl2plan)>1.0)&&u<1-0.0001&&Vel_flag>0) 
    {
      
      Pc=getCurvePoint(u,count, U, fitPoint);
      d_amcl2goal=getDistance(amcl_pose,Pc);

      angle_plan=atan2(Pc.Y-amcl_pose.Y,Pc.X-amcl_pose.X);
      angle_amcl2plan=angle_diff(angle_plan,amcl_pose.Z);
      u+=0.0001;
      Pc2=getCurvePoint(u,count, U, fitPoint);
      Pc.Z=atan2(Pc2.Y-Pc.Y,Pc2.X-Pc.X);
      angle_amcl2plan2=angle_diff(Pc.Z,amcl_pose.Z);
     angel_flag=(Pc.X-amcl_pose.X)*(amcl_pose.X-last_pose.X)+(Pc.Y-amcl_pose.Y)*(amcl_pose.Y-last_pose.Y);
     angel_flag2=(Pc.X-amcl_pose.X)*(Pc2.X-Pc.X)+(Pc.Y-amcl_pose.Y)*(Pc2.Y-Pc.Y);
      
     ROS_INFO("u=%.4f angel_flag=%.4f angel_flag2=%.4f d=%.2f",u,angel_flag,angel_flag2,d_amcl2goal);
    }
    angle_plan=beta*angle_amcl2plan+(1-beta)*angle_amcl2plan2/2;
    cmdVel.linear.x=0.7; 
    cmdVel.angular.z=cmdVel.linear.x*(sin(angle_plan))/(d_amcl2goal/2);
   
    ROS_INFO("u=%.4f linear.x=%.2f angular.z=%.4f",u,cmdVel.linear.x,cmdVel.angular.z);
    ROS_INFO("W=%.4f theta=%.2f theta2=%.2f,angle_plan=%.2f",cmdVel.angular.z,angle_amcl2plan,angle_amcl2plan2,angle_plan);

    outStream<<"W="<<cmdVel.angular.z<<" theta="<<angle_amcl2plan<<" theta2="<<angle_amcl2plan2<<" angle_plan="<<angle_plan<<"  \n";
    cmd_pub.publish(cmdVel);
    
   }

   
  
    
   ros::spinOnce();
   loop_rate.sleep();

  }

  outStream.close();
  return 0;
}
