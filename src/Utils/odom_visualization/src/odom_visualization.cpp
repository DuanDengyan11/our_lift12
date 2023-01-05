#include <iostream>
#include <string>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Range.h"
#include "visualization_msgs/Marker.h"
#include "armadillo"
#include "pose_utils.h"
#include "quadrotor_msgs/PositionCommand.h"

using namespace arma;
using namespace std;

static  string mesh_resource_load, mesh_resource_heli1, mesh_resource_heli2;
static  string mesh_resource_heli3, mesh_resource_heli4;
static double color_r, color_g, color_b, color_a;
static double scale, scale1, scale2, scale3, scale4;

double cross_yaw, cross_pitch, cross_roll, cross_x, cross_y, cross_z;
double cross_yaw1, cross_pitch1, cross_roll1, cross_x1, cross_y1, cross_z1;
double cross_yaw2, cross_pitch2, cross_roll2, cross_x2, cross_y2, cross_z2;
double cross_yaw3, cross_pitch3, cross_roll3, cross_x3, cross_y3, cross_z3;
double cross_yaw4, cross_pitch4, cross_roll4, cross_x4, cross_y4, cross_z4;

bool   cross_config = false;
bool   origin       = false;
bool   isOriginSet  = false;
colvec poseOrigin(6);

ros::Publisher pathPub;
ros::Publisher meshPub;

ros::Publisher pathPub1;
ros::Publisher meshPub1;

ros::Publisher pathPub2;
ros::Publisher meshPub2;

ros::Publisher pathPub3;
ros::Publisher meshPub3;

ros::Publisher pathPub4;
ros::Publisher meshPub4;

tf::TransformBroadcaster* broadcaster;
string _frame_id;
int _drone_id;

  geometry_msgs::PoseStamped poseROS1;
  nav_msgs::Path             pathROS1;
  visualization_msgs::Marker meshROS1;
void odom_callback1(const nav_msgs::Odometry::ConstPtr& msg)
{
   if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS1.header = msg->header;
  poseROS1.header.stamp = msg->header.stamp;
  poseROS1.header.frame_id = string("world");
  poseROS1.pose.position.x = pose(0);
  poseROS1.pose.position.y = pose(1);
  poseROS1.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS1.pose.orientation.w = q(0);
  poseROS1.pose.orientation.x = q(1);
  poseROS1.pose.orientation.y = q(2);
  poseROS1.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS1.header = poseROS1.header;
    pathROS1.poses.push_back(poseROS1);
    pathPub1.publish(pathROS1);
  }

  // Mesh model                                                  
  meshROS1.header.frame_id = _frame_id;
  meshROS1.header.stamp = msg->header.stamp; 
  meshROS1.ns = "mesh";
  meshROS1.id = 0;
  meshROS1.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS1.action = visualization_msgs::Marker::ADD;
  meshROS1.pose.position.x = msg->pose.pose.position.x + cross_x1;
  meshROS1.pose.position.y = msg->pose.pose.position.y + cross_y1;
  meshROS1.pose.position.z = msg->pose.pose.position.z + cross_z1;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw1*PI/180.0;
    ypr(1)    += cross_pitch1*PI/180.0;
    ypr(2)    += cross_roll1*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS1.pose.orientation.w = q(0);
  meshROS1.pose.orientation.x = q(1);
  meshROS1.pose.orientation.y = q(2);
  meshROS1.pose.orientation.z = q(3);
  meshROS1.scale.x = scale1;
  meshROS1.scale.y = scale1;
  meshROS1.scale.z = scale1;
  meshROS1.color.a = color_a;
  meshROS1.color.r = color_r;
  meshROS1.color.g = color_g;
  meshROS1.color.b = color_b;
  meshROS1.mesh_resource = mesh_resource_heli1;
  meshPub1.publish(meshROS1);

}

geometry_msgs::PoseStamped poseROS2;
nav_msgs::Path             pathROS2;
visualization_msgs::Marker meshROS2;
void odom_callback2(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS2.header = msg->header;
  poseROS2.header.stamp = msg->header.stamp;
  poseROS2.header.frame_id = string("world");
  poseROS2.pose.position.x = pose(0);
  poseROS2.pose.position.y = pose(1);
  poseROS2.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS2.pose.orientation.w = q(0);
  poseROS2.pose.orientation.x = q(1);
  poseROS2.pose.orientation.y = q(2);
  poseROS2.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS2.header = poseROS2.header;
    pathROS2.poses.push_back(poseROS2);
    pathPub2.publish(pathROS2);
  }

  // Mesh model                                                  
  meshROS2.header.frame_id = _frame_id;
  meshROS2.header.stamp = msg->header.stamp; 
  meshROS2.ns = "mesh";
  meshROS2.id = 0;
  meshROS2.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS2.action = visualization_msgs::Marker::ADD;
  meshROS2.pose.position.x = msg->pose.pose.position.x + cross_x2;
  meshROS2.pose.position.y = msg->pose.pose.position.y + cross_y2;
  meshROS2.pose.position.z = msg->pose.pose.position.z + cross_z2;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw2*PI/180.0;
    ypr(1)    += cross_pitch2*PI/180.0;
    ypr(2)    += cross_roll2*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS2.pose.orientation.w = q(0);
  meshROS2.pose.orientation.x = q(1);
  meshROS2.pose.orientation.y = q(2);
  meshROS2.pose.orientation.z = q(3);
  meshROS2.scale.x = scale2;
  meshROS2.scale.y = scale2;
  meshROS2.scale.z = scale2;
  meshROS2.color.a = color_a;
  meshROS2.color.r = 1.0;
  meshROS2.color.g = 0.0;
  meshROS2.color.b = 0.0;
  meshROS2.mesh_resource = mesh_resource_heli2;
  meshPub2.publish(meshROS2);

}

geometry_msgs::PoseStamped poseROS3;
nav_msgs::Path             pathROS3;
visualization_msgs::Marker meshROS3;
void odom_callback3(const nav_msgs::Odometry::ConstPtr& msg)
{

    if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS3.header = msg->header;
  poseROS3.header.stamp = msg->header.stamp;
  poseROS3.header.frame_id = string("world");
  poseROS3.pose.position.x = pose(0);
  poseROS3.pose.position.y = pose(1);
  poseROS3.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS3.pose.orientation.w = q(0);
  poseROS3.pose.orientation.x = q(1);
  poseROS3.pose.orientation.y = q(2);
  poseROS3.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS3.header = poseROS3.header;
    pathROS3.poses.push_back(poseROS3);
    pathPub3.publish(pathROS3);
  }

  // Mesh model                                                  
  meshROS3.header.frame_id = _frame_id;
  meshROS3.header.stamp = msg->header.stamp; 
  meshROS3.ns = "mesh";
  meshROS3.id = 0;
  meshROS3.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS3.action = visualization_msgs::Marker::ADD;
  meshROS3.pose.position.x = msg->pose.pose.position.x + cross_x3;
  meshROS3.pose.position.y = msg->pose.pose.position.y + cross_y3;
  meshROS3.pose.position.z = msg->pose.pose.position.z + cross_z3;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw3*PI/180.0;
    ypr(1)    += cross_pitch3*PI/180.0;
    ypr(2)    += cross_roll3*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS3.pose.orientation.w = q(0);
  meshROS3.pose.orientation.x = q(1);
  meshROS3.pose.orientation.y = q(2);
  meshROS3.pose.orientation.z = q(3);
  meshROS3.scale.x = scale3;
  meshROS3.scale.y = scale3;
  meshROS3.scale.z = scale3;
  meshROS3.color.a = color_a;
  meshROS3.color.r = 0.0;
  meshROS3.color.g = 1.0;
  meshROS3.color.b = 0.0;
  meshROS3.mesh_resource = mesh_resource_heli3;
  meshPub3.publish(meshROS3);

}

geometry_msgs::PoseStamped poseROS4;
nav_msgs::Path             pathROS4;
visualization_msgs::Marker meshROS4;
void odom_callback4(const nav_msgs::Odometry::ConstPtr& msg)
{

    if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS4.header = msg->header;
  poseROS4.header.stamp = msg->header.stamp;
  poseROS4.header.frame_id = string("world");
  poseROS4.pose.position.x = pose(0);
  poseROS4.pose.position.y = pose(1);
  poseROS4.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS4.pose.orientation.w = q(0);
  poseROS4.pose.orientation.x = q(1);
  poseROS4.pose.orientation.y = q(2);
  poseROS4.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS4.header = poseROS4.header;
    pathROS4.poses.push_back(poseROS4);
    pathPub4.publish(pathROS4);
  }

  // Mesh model                                                  
  meshROS4.header.frame_id = _frame_id;
  meshROS4.header.stamp = msg->header.stamp; 
  meshROS4.ns = "mesh";
  meshROS4.id = 0;
  meshROS4.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS4.action = visualization_msgs::Marker::ADD;
  meshROS4.pose.position.x = msg->pose.pose.position.x + cross_x4;
  meshROS4.pose.position.y = msg->pose.pose.position.y + cross_y4;
  meshROS4.pose.position.z = msg->pose.pose.position.z + cross_z4;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw4*PI/180.0;
    ypr(1)    += cross_pitch4*PI/180.0;
    ypr(2)    += cross_roll4*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS4.pose.orientation.w = q(0);
  meshROS4.pose.orientation.x = q(1);
  meshROS4.pose.orientation.y = q(2);
  meshROS4.pose.orientation.z = q(3);
  meshROS4.scale.x = scale4;
  meshROS4.scale.y = scale4;
  meshROS4.scale.z = scale4;
  meshROS4.color.a = color_a;
  meshROS4.color.r = 0.0;
  meshROS4.color.g = 0.0;
  meshROS4.color.b = 1.0;
  meshROS4.mesh_resource = mesh_resource_heli4;
  meshPub4.publish(meshROS4);

}

    geometry_msgs::PoseStamped poseROS;
    nav_msgs::Path             pathROS;
    visualization_msgs::Marker meshROS;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

  if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub.publish(pathROS);
  }

  // Mesh model                                                  
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp; 
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x + cross_x;
  meshROS.pose.position.y = msg->pose.pose.position.y + cross_y;
  meshROS.pose.position.z = msg->pose.pose.position.z + cross_z;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw*PI/180.0;
    ypr(1)    += cross_pitch*PI/180.0;
    ypr(2)    += cross_roll*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource_load;
  meshPub.publish(meshROS);                                                  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle n("~");

  n.param("mesh_resource_load", mesh_resource_load, std::string("package://odom_visualization/meshes/load.stl"));
  n.param("mesh_resource_heli1", mesh_resource_heli1, std::string("package://odom_visualization/meshes/heli.obj"));
  n.param("mesh_resource_heli2", mesh_resource_heli2, std::string("package://odom_visualization/meshes/heli.obj"));
  n.param("mesh_resource_heli3", mesh_resource_heli3, std::string("package://odom_visualization/meshes/heli.obj"));
  n.param("mesh_resource_heli4", mesh_resource_heli4, std::string("package://odom_visualization/meshes/heli.obj"));

  n.param("color/r", color_r, 1.0);
  n.param("color/g", color_g, 0.0);
  n.param("color/b", color_b, 0.0);
  n.param("color/a", color_a, 1.0);
  n.param("origin", origin, false);  
  n.param("frame_id",   _frame_id, string("world") ); 
  n.param("cross_config", cross_config, false);    

  n.param("robot_scale", scale, 2.0);    
  n.param("cross_yaw", cross_yaw, 0.0);
  n.param("cross_pitch", cross_pitch, 0.0);
  n.param("cross_roll", cross_roll, 0.0);
  n.param("cross_x", cross_x, 0.0);
  n.param("cross_y", cross_y, 0.0);
  n.param("cross_z", cross_z, 0.0);
  
  ros::Subscriber sub_odom = n.subscribe("odom", 100,  odom_callback);
  ros::Subscriber sub_odom1 = n.subscribe("odom1", 100,  odom_callback1);
  ros::Subscriber sub_odom2 = n.subscribe("odom2", 100,  odom_callback2);
  ros::Subscriber sub_odom3 = n.subscribe("odom3", 100,  odom_callback3);
  ros::Subscriber sub_odom4 = n.subscribe("odom4", 100,  odom_callback4);

  //load 
  pathPub   = n.advertise<nav_msgs::Path>(            "path",                100, true);
  meshPub   = n.advertise<visualization_msgs::Marker>("robot",               100, true);  

  //heli1
  meshPub1   = n.advertise<visualization_msgs::Marker>("robot1",               100, true);  
  pathPub1   = n.advertise<nav_msgs::Path>(            "path1",                100, true);
  n.param("robot_scale1", scale1, 2.0);    
  n.param("cross_yaw1", cross_yaw1, 0.0);
  n.param("cross_pitch1", cross_pitch1, 0.0);
  n.param("cross_roll1", cross_roll1, 0.0);
  n.param("cross_x1", cross_x1, 0.0);
  n.param("cross_y1", cross_y1, 0.0);
  n.param("cross_z1", cross_z1, 0.0);

  //heli2
  meshPub2   = n.advertise<visualization_msgs::Marker>("robot2",               100, true);  
  pathPub2   = n.advertise<nav_msgs::Path>(            "path2",                100, true);
  n.param("robot_scale2", scale2, 2.0);    
  n.param("cross_yaw2", cross_yaw2, 0.0);
  n.param("cross_pitch2", cross_pitch2, 0.0);
  n.param("cross_roll2", cross_roll2, 0.0);
  n.param("cross_x2", cross_x2, 0.0);
  n.param("cross_y2", cross_y2, 0.0);
  n.param("cross_z2", cross_z2, 0.0);

  //heli3
  meshPub3   = n.advertise<visualization_msgs::Marker>("robot3",               100, true);  
  pathPub3   = n.advertise<nav_msgs::Path>(            "path3",                100, true);
  n.param("robot_scale3", scale3, 2.0);    
  n.param("cross_yaw3", cross_yaw3, 0.0);
  n.param("cross_pitch3", cross_pitch3, 0.0);
  n.param("cross_roll3", cross_roll3, 0.0);
  n.param("cross_x3", cross_x3, 0.0);
  n.param("cross_y3", cross_y3, 0.0);
  n.param("cross_z3", cross_z3, 0.0);

  //heli4
  meshPub4   = n.advertise<visualization_msgs::Marker>("robot4",               100, true);  
  pathPub4   = n.advertise<nav_msgs::Path>(            "path4",                100, true);
  n.param("robot_scale4", scale4, 2.0);    
  n.param("cross_yaw4", cross_yaw4, 0.0);
  n.param("cross_pitch4", cross_pitch4, 0.0);
  n.param("cross_roll4", cross_roll4, 0.0);
  n.param("cross_x4", cross_x4, 0.0);
  n.param("cross_y4", cross_y4, 0.0);
  n.param("cross_z4", cross_z4, 0.0);

  tf::TransformBroadcaster b;
  broadcaster = &b;

  ros::spin();

  return 0;
}
