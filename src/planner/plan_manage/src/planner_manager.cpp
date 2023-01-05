// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager()
  {
    std::cout << "des manager" << std::endl;
  }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);
    nh.param("cable/n_coef_iterate", n_coef_iterate_, 1);
  

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    cable_load_.reset(new cable_load);
    cable_load_->init(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_, cable_load_);

    visualization_ = vis;

  }

  bool EGOPlannerManager::computeInitReferenceState(const Eigen::VectorXd &start_pt,
                                                    const Eigen::VectorXd &start_vel,
                                                    const Eigen::VectorXd &start_acc,
                                                    const Eigen::VectorXd &local_target_pt,
                                                    const Eigen::VectorXd &local_target_vel,
                                                    poly_traj::MinJerkOpt &initMJO,
                                                    const bool flag_polyInit)
  {
    static bool flag_first_call = true;

    /*** case 1: use A* initialization ***/
    if (flag_first_call || flag_polyInit)
    {
      flag_first_call = false;
      /* basic params */
      Eigen::Matrix<double, 12, 3> headState, tailState;
      vector<Eigen::Vector3d> simple_path;

      headState << start_pt, start_vel, start_acc; //初始状态
      tailState << local_target_pt, local_target_vel, Eigen::Matrix<double, 12, 1>::Zero(); //目标状态

      /* step 1: A* search and gWenerate init traj */
      Eigen::MatrixXd ctl_points;

      // traj = ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points);
      //采用Astar算法产生初始轨迹
      ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points, initMJO);

      // show the init simple_path
      vector<vector<Eigen::Vector3d>> path_view;
      path_view.push_back(simple_path);
      visualization_->displayAStarList(path_view, 0);

      // show the init traj for debug
      std::vector<Eigen::Vector3d> point_set;
      for (int i = 0; i < ctl_points.cols(); ++i)
        point_set.push_back(ctl_points.col(i).topRows(3));
      visualization_->displayInitPathListDebug(point_set, 0.2, 0);
    }

    /*** case 2: initialize from previous optimal trajectory ***/
    else
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).topRows(3).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix<double, 12, 3> headState, tailState;
      Eigen::MatrixXd innerPs(12, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Matrix<double, 12, 1>::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88");
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel)
  {
    double t;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;  //planning_horizen = 7.5m
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time).topRows(3);
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time).topRows(3);
    }
  }

  //for load
  bool EGOPlannerManager::reboundReplan(
      const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc,
      const double trajectory_start_time, const Eigen::VectorXd &local_target_pt, const Eigen::VectorXd &local_target_vel,
      const bool flag_polyInit,  const bool have_local_traj)
  {
    static int count = 0;

    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);

    if ((start_pt - local_target_pt).topRows(3).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/   
    poly_traj::MinJerkOpt initMJO;
    if (!computeInitReferenceState(start_pt, start_vel, start_acc,
                                   local_target_pt, local_target_vel,
                                  initMJO, flag_polyInit))
    {
      return false;
    }


    Eigen::MatrixXd cstr_pts = initMJO.getInitConstrainPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    ploy_traj_opt_->setControlPoints(cstr_pts);

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i).topRows(3));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    poly_traj::Trajectory initTraj = initMJO.getTraj();
    int PN = initTraj.getPieceNum();
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    Eigen::MatrixXd innerPts = all_pos.block(0, 1, 12, PN - 1);
    Eigen::Matrix<double, 12, 3> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
    tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
    
    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(headState, tailState,
                                                            innerPts, initTraj.getDurations(),
                                                            cstr_pts, trajectory_start_time);
 
    t_opt = ros::Time::now() - t_start;

    if (!flag_success)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt).toSec()
         << "\033[0m,init:" << t_init.toSec()
         << ",optimize:" << t_opt.toSec()
         << ",avg_time=" << sum_time / count_success
         << ",count_success= " << count_success << endl;
    average_plan_time_ = sum_time / count_success;

    visualization_->displayOptimalList(cstr_pts, 0);

    if(have_local_traj)
    {
      double delta_replan_time = trajectory_start_time - ros::Time::now().toSec();
      cout << "time" << delta_replan_time << endl;
      if (delta_replan_time > 0)
        ros::Duration(delta_replan_time).sleep();
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), trajectory_start_time);
    }else{
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), ros::Time::now().toSec()); // todo time
    }

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }


  std::vector<std::vector<Eigen::VectorXd>> EGOPlannerManager::getResults()
  {
    return ploy_traj_opt_->results_total;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    poly_traj::MinJerkOpt globalMJO;
    
    //for waypoints
    int N = waypoints.size();

    //cable coefficients equal to zero when acc = 0 and the cable force only to trim gravity force
    // odom_pos_ + cable_coef;
    Eigen::Matrix<double, 12, 1> pos_sys_start, vel_sys_start, acc_sys_start;
    Eigen::Matrix<double, 12, 1> pos_sys_end, vel_sys_end, acc_sys_end;
    pos_sys_start << start_pos, cable_load_->init_Tq;
    vel_sys_start << start_vel, Eigen::Matrix<double, 9, 1>::Zero();
    acc_sys_start << start_acc, Eigen::Matrix<double, 9, 1>::Zero();
    pos_sys_end << waypoints.back(), cable_load_->init_Tq;
    vel_sys_end << end_vel, Eigen::Matrix<double, 9, 1>::Zero();
    acc_sys_end << end_acc, Eigen::Matrix<double, 9, 1>::Zero();

    Eigen::Matrix<double, 12, 3> headState, tailState;
    headState << pos_sys_start, vel_sys_start, acc_sys_start;
    tailState << pos_sys_end, vel_sys_end, acc_sys_end;

    Eigen::MatrixXd innerPts;

    if (N > 1)
    {
      innerPts.resize(12, N - 1);
      innerPts.setZero();
      for (int i = 0; i < N - 1; i++)
        innerPts.block<3,1>(0,i) = waypoints[i];
    } 
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }


    globalMJO.reset(headState, tailState, N);

    double des_vel = pp_.max_vel_;
    Eigen::VectorXd time_vec(N);
    int try_num = 0;
    do
    {
      for (int i = 0; i < N; ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }
      globalMJO.generate(innerPts, time_vec);
      // cout << "try_num : " << try_num << endl;
      // cout << "max vel : " << globalMJO.getTraj().getMaxVelRate() << endl;
      // cout << "time_vec : " << time_vec.transpose() << endl;

      des_vel /= 1.2;
      try_num++;
    } while (globalMJO.getTraj().getMaxVelRate() > pp_.max_vel_ && try_num <= 5);

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace ego_planner
