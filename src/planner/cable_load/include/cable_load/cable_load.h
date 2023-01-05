#include<iostream>
using namespace std;
#include <vector>
#include <Eigen/Eigen> 
#include <ros/ros.h>

class cable_load
{
private:

    double grav = 9.8;
    ros::NodeHandle node_;
    
public:
    cable_load(){};
    ~cable_load(){};
    void init(ros::NodeHandle &nh);

    Eigen::Matrix3d Transformb2e;
    Eigen::Matrix<double, 9, 1> init_Tq;  // alpha = 30 degree beta = 45 degree

    double cable_length_, load_mass_;

    void compute_init_Tq();
    void compute_Tq1(Eigen::Vector3d acc_load, Eigen::VectorXd Tq22n, Eigen::Vector3d &Tq1);
    void compute_uav_pos(Eigen::VectorXd FMeach, Eigen::Vector3d load_position, std::vector<Eigen::Vector3d> &uav_positions);

	typedef std::shared_ptr<cable_load> Ptr;

};
