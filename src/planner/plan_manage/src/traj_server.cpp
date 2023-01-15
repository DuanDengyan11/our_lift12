using namespace std;
#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <iostream>
#include <cable_load/cable_load.h>

fstream result_file_;
ros::Publisher pos_cmd_pub, pos_cmd_pub1, pos_cmd_pub2, pos_cmd_pub3, pos_cmd_pub4;

quadrotor_msgs::PositionCommand cmd, cmd1, cmd2, cmd3, cmd4;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time time_last;

double time_forward_;

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    cMats[i].row(3) << msg->coef_tq2x[i6 + 0], msg->coef_tq2x[i6 + 1], msg->coef_tq2x[i6 + 2],
        msg->coef_tq2x[i6 + 3], msg->coef_tq2x[i6 + 4], msg->coef_tq2x[i6 + 5];
    cMats[i].row(4) << msg->coef_tq2y[i6 + 0], msg->coef_tq2y[i6 + 1], msg->coef_tq2y[i6 + 2],
        msg->coef_tq2y[i6 + 3], msg->coef_tq2y[i6 + 4], msg->coef_tq2y[i6 + 5];
    cMats[i].row(5) << msg->coef_tq2z[i6 + 0], msg->coef_tq2z[i6 + 1], msg->coef_tq2z[i6 + 2],
        msg->coef_tq2z[i6 + 3], msg->coef_tq2z[i6 + 4], msg->coef_tq2z[i6 + 5];

    cMats[i].row(6) << msg->coef_tq3x[i6 + 0], msg->coef_tq3x[i6 + 1], msg->coef_tq3x[i6 + 2],
        msg->coef_tq3x[i6 + 3], msg->coef_tq3x[i6 + 4], msg->coef_tq3x[i6 + 5];
    cMats[i].row(7) << msg->coef_tq3y[i6 + 0], msg->coef_tq3y[i6 + 1], msg->coef_tq3y[i6 + 2],
        msg->coef_tq3y[i6 + 3], msg->coef_tq3y[i6 + 4], msg->coef_tq3y[i6 + 5];
    cMats[i].row(8) << msg->coef_tq3z[i6 + 0], msg->coef_tq3z[i6 + 1], msg->coef_tq3z[i6 + 2],
        msg->coef_tq3z[i6 + 3], msg->coef_tq3z[i6 + 4], msg->coef_tq3z[i6 + 5];

    cMats[i].row(9) << msg->coef_tq4x[i6 + 0], msg->coef_tq4x[i6 + 1], msg->coef_tq4x[i6 + 2],
        msg->coef_tq4x[i6 + 3], msg->coef_tq4x[i6 + 4], msg->coef_tq4x[i6 + 5];
    cMats[i].row(10) << msg->coef_tq4y[i6 + 0], msg->coef_tq4y[i6 + 1], msg->coef_tq4y[i6 + 2],
        msg->coef_tq4y[i6 + 3], msg->coef_tq4y[i6 + 4], msg->coef_tq4y[i6 + 5];
    cMats[i].row(11) << msg->coef_tq4z[i6 + 0], msg->coef_tq4z[i6 + 1], msg->coef_tq4z[i6 + 2],
        msg->coef_tq4z[i6 + 3], msg->coef_tq4z[i6 + 4], msg->coef_tq4z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));
  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  time_last = ros::Time::now();
  receive_traj_ = true;


}

void pubCmd(cable_load cable_load_)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();


  Eigen::Vector3d pos_load(Eigen::Vector3d::Zero()), vel_load(Eigen::Vector3d::Zero()), acc_load(Eigen::Vector3d::Zero());
  Eigen::VectorXd pos_tq22n(9), vel_tq22n(9), acc_tq22n(9);
  std::pair<double, double> yaw_yawdot(0, 0);

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos_load = traj_->getPos(t_cur).topRows(3);
    vel_load = traj_->getVel(t_cur).topRows(3);
    acc_load = traj_->getAcc(t_cur).topRows(3);

    pos_tq22n = traj_->getPos(t_cur).bottomRows(9);
    vel_tq22n = traj_->getVel(t_cur).bottomRows(9);
    acc_tq22n = traj_->getAcc(t_cur).bottomRows(9);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos_load = traj_->getPos(traj_duration_).topRows(3);
    vel_load.setZero();
    acc_load.setZero();

    pos_tq22n = traj_->getPos(traj_duration_).bottomRows(9);
    vel_tq22n.setZero();
    acc_tq22n.setZero();
  }
  else
  {
    // std::cout << "[Traj server]: invalid time." << std::endl;
  }

  // cout << "t_cur" << t_cur << "duration" << traj_duration_ << "pos_load" << pos_load(0) << pos_load(1) << pos_load(2) << endl;

    for(int i=0; i < 3; i++)
  {
    result_file_ << pos_load(i) << '\t';
  }

  for(int i=0; i<9; i++)
  {
    result_file_ << pos_tq22n(i) << '\t';
  }

  for(int i=0; i < 3; i++)
  {
    result_file_ << vel_load(i) << '\t';
  }

  for(int i=0; i<9; i++)
  {
    result_file_ << vel_tq22n(i) << '\t';
  }


  for(int i=0; i < 3; i++)
  {
    result_file_ << acc_load(i) << '\t';
  }

  for(int i=0; i<9; i++)
  {
    result_file_ << acc_tq22n(i) << '\t';
  }

  result_file_ << '\n';

  // load 
  yaw_yawdot.first = 0;
  yaw_yawdot.second = 0;
  time_last = time_now;
  
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos_load(0);
  cmd.position.y = pos_load(1);
  cmd.position.z = pos_load(2);
  
  cmd.velocity.x = vel_load(0);
  cmd.velocity.y = vel_load(1);
  cmd.velocity.z = vel_load(2);

  cmd.acceleration.x = acc_load(0);
  cmd.acceleration.y = acc_load(1);
  cmd.acceleration.z = acc_load(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;
  pos_cmd_pub.publish(cmd);

  //compute FMeach
  Eigen::VectorXd FMeach(12);
  Eigen::Vector3d tq1;
  cable_load_.compute_Tq1(acc_load, pos_tq22n, tq1);
  FMeach << tq1, pos_tq22n;

  //compute uav positions
  std::vector<Eigen::Vector3d> uav_positions;
  cable_load_.compute_uav_pos(FMeach, pos_load, uav_positions);
  
  // publish uav position
  cmd1 = cmd;
  cmd1.position.x = uav_positions[0](0);
  cmd1.position.y = uav_positions[0](1);
  cmd1.position.z = uav_positions[0](2);
  pos_cmd_pub1.publish(cmd1);

  cmd2 = cmd;
  cmd2.position.x = uav_positions[1](0);
  cmd2.position.y = uav_positions[1](1);
  cmd2.position.z = uav_positions[1](2);
  pos_cmd_pub2.publish(cmd2);

  cmd3 = cmd;
  cmd3.position.x = uav_positions[2](0);
  cmd3.position.y = uav_positions[2](1);
  cmd3.position.z = uav_positions[2](2);
  pos_cmd_pub3.publish(cmd3);

  cmd4 = cmd;
  cmd4.position.x = uav_positions[3](0);
  cmd4.position.y = uav_positions[3](1);
  cmd4.position.z = uav_positions[3](2);
  pos_cmd_pub4.publish(cmd4);
  
  // cout << "acc" << acc_load.transpose() << "uav0" << uav_positions[0].transpose() << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~"); //发布的话题名为节点名/定义的话题名
  
  cable_load cable_load_;
  cable_load_.init(nh);

  ros::Subscriber poly_traj_sub0 = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  pos_cmd_pub1 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd1", 50);
  pos_cmd_pub2 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd2", 50);
  pos_cmd_pub3 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd3", 50);
  pos_cmd_pub4 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd4", 50);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  ROS_WARN("[Traj server]: ready.");

  time_t t;
  struct tm *tmp;
  time(&t);
  tmp = localtime(&t);
  char buf2[128];
  strftime(buf2, 64, "/home/tutu/our_lift12/traj_%Y-%m-%d %H:%M:%S.txt", tmp);
  result_file_.open(buf2, ios::app);
  
  ros::Rate rate(100);
  bool status = ros::ok();
  while(status)
  {
    pubCmd(cable_load_);
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
  return 0;
}