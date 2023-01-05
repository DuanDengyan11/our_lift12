#include <cable_load/cable_load.h>

void cable_load::init(ros::NodeHandle &nh)
{  
    node_ = nh;
    node_.param("optimization/load_mass", load_mass_, 2.0); // 吊挂物质量
    node_.param("cable/cable_length", cable_length_, 2.0); // cable length

    Transformb2e << 1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0;

    compute_init_Tq();
}

void cable_load::compute_init_Tq()
{
    double alpha = M_PI/6.0, beta = M_PI/4.0, weight = load_mass_ * grav;
    double temp1 = weight/4.0 * tan(alpha)*cos(beta);
    double temp2 = weight/4.0 * tan(alpha)*sin(beta);
    double temp3 = weight/4.0;

    init_Tq << -temp1, temp2, -temp3, temp1, temp2, -temp3, temp1, -temp2, -temp3; 
}

void cable_load::compute_Tq1(Eigen::Vector3d acc_load, Eigen::VectorXd Tq22n, Eigen::Vector3d &Tq1)
{
    Eigen::Vector3d Tqn = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < 3; i++)
    {
        Tqn += Tq22n.block<3,1>(3*i, 0);
    }
    
    Eigen::VectorXd acc(3);
    acc << acc_load(0), -acc_load(1), -acc_load(2) - grav;

    Tq1 = load_mass_ * acc - Tqn;
}


void cable_load::compute_uav_pos(Eigen::VectorXd FMeach, Eigen::Vector3d load_position, std::vector<Eigen::Vector3d> &uav_positions)
{
    uav_positions.clear();
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
      Eigen::Vector3d qi = FMi / FMi.norm();
      Eigen::Vector3d uav_position = load_position + Transformb2e *(cable_length_ * qi);
      uav_positions.push_back(uav_position); // uav position 2 - n
    }
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ego_planner_node");
//     ros::NodeHandle nh;
//     cable_load cable_load_;
//     cable_load_.init(nh);

//     Eigen::VectorXd Tq22n =  cable_load_.init_Tq;
//     Eigen::Vector3d acc_load =  Eigen::Vector3d::Zero();
//     Eigen::Vector3d load_position = Eigen::Vector3d::Zero();
//     std::vector<Eigen::Vector3d> uav_positions;

//     cable_load_.compute_uav_pos(Tq22n, acc_load, load_position, uav_positions);
//     for (size_t i = 0; i < uav_positions.size(); i++)
//     {
//         cout << uav_positions[i].transpose() << endl;
//     }    
// }