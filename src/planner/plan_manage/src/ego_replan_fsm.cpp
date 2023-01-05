

#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{
  EGOReplanFSM::~EGOReplanFSM()
  {
    result_file_.close();
  }
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;  //初始设置执行器为INIT
 
    have_target_ = false; //是否接收到目标位置点
    have_odom_ = false;  //是否有里程计数据
    have_odom_1 = false;  //是否有里程计数据
    have_odom_2 = false;  //是否有里程计数据
    have_odom_3 = false;  //是否有里程计数据
    have_odom_4 = false;  //是否有里程计数据

    have_local_traj_ = false; // 有没有上次规划好的local load轨迹
    flag_relan_astar_ = true; //是否需要重新astar规划

    /*  fsm param  */
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);  // = 1s
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0); // = 1m
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0); // = 7.5m 预测距离
    nh.param("fsm/result_file", result_fn_, string("/home/zuzu/Documents/Benchmark/21-RSS-ego-swarm/2.24/ego/ego_swarm.txt")); // 保存结果文件的地方
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0); // = 0.1s 重新规划算法的时间

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this); //主程序 先优化吊挂物轨迹 然后优化上面的绳索
    // exec_timer_plot_figure = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::plotfigure, this); 


    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this); //接收到的odom消息，our_lift中我们设定为吊挂物当前位置

    //要发给odom的 
    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);

    //记录优化结果
    result_file_.open(result_fn_, ios::app);

    central_goal = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::formationWaypointCallback, this); //地图上鼠标发来的目标位置

  }

  void EGOReplanFSM::plotfigure()
  {
    // plt::figure(1);
    std::map<int, std::string> ylabel_ = {{0, "smooth_cost"}, 
      {1, "time_cost"},
      {2, "load_obstacle"},
      {3, "load_feasibility"}, 
      {4, "uav_swarm"},
      {5, "uav_obstacle"},
      {6, "cable_collision"},
      {7, "cable_feasibility"},
      {8, "quadratic"},
      {9, "total"}};

    std::vector<std::vector<Eigen::VectorXd>> results_total = planner_manager_->getResults();
    std::vector<Eigen::VectorXd> results = results_total[0];

    plt::figure_size(1200,780);
    
    size_t iter_num = results.size();
    size_t variable_num = results[0].size(); 
    long fig_num = variable_num - 1;

    // columns = 3
    // long fig_rows = ceil(fig_num / fig_columns);
    // plt::figure_size(1800, 1200);

    std::vector<std::vector<double>> datas;
    std::vector<double> data;
    cout << "size" << results_total.size() << endl;

    for(size_t j = 0; j < variable_num; j++)
    {
      data.clear();
      for(size_t k = 0; k < results_total.size(); k++)
      {
        results = results_total[k];
        iter_num = results.size();
        for (size_t i = 0; i < iter_num; i++)
        {
          data.push_back(results[i](j)); 
        }
      }
      datas.push_back(data);
    }

    for (int i = 0; i < fig_num; i++)
    {
        plt::subplot(3, 4, i+1);
        plt::plot( datas[0], datas[i+1]);
        plt::ylabel(ylabel_[i]);
    }
    
    std::map<std::string, double> m = {{"hspace", 0.3}, {"wspace", 0.45}};
    plt::subplots_adjust(m);

    plt::show();
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      bool success = planFromGlobalTraj(1);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        ROS_ERROR("Failed to generate the first trajectory!!!");
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      bool success;
      if (flag_relan_astar_)
        success = planFromLocalTraj(true);
      else
        success = planFromLocalTraj(false);

      if (success) //若成功，就不需要重新astar计算了
      {
        flag_relan_astar_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        flag_relan_astar_ = true;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur).topRows(3);

      if ((local_target_pt_ - end_pt_).norm() < 0.1) // local target close to the global target
      {
        if (t_cur > info->duration - 0.2)
        {
          have_target_ = false;
          have_local_traj_ = false;

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");

          result_file_ << planner_manager_->pp_.drone_id << "\t" << (ros::Time::now() - planner_manager_->global_start_time_).toSec() << "\t" << planner_manager_->average_plan_time_ << "\n";

          printf("\033[47;30m\n[drone %d reached goal]==============================================\033[0m\n",
                 planner_manager_->pp_.drone_id);
          
          plotfigure();
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
 
      break;
    }
    }

    force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;

  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &msg)
  {
    auto data = &planner_manager_->traj_.local_traj;
    msg.drone_id = 0;
    msg.traj_id = data->traj_id;
    msg.start_time = ros::Time(data->start_time);
    msg.order = 5; // todo, only support order = 5 now.

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    msg.coef_tq2x.resize(6*piece_num);
    msg.coef_tq2y.resize(6*piece_num);
    msg.coef_tq2z.resize(6*piece_num);
    msg.coef_tq3x.resize(6*piece_num);
    msg.coef_tq3y.resize(6*piece_num);
    msg.coef_tq3z.resize(6*piece_num);
    msg.coef_tq4x.resize(6*piece_num);
    msg.coef_tq4y.resize(6*piece_num);
    msg.coef_tq4z.resize(6*piece_num);

    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
        msg.coef_tq2x[i6 + j] = cMat(3, j);
        msg.coef_tq2y[i6 + j] = cMat(4, j);
        msg.coef_tq2z[i6 + j] = cMat(5, j);
        msg.coef_tq3x[i6 + j] = cMat(6, j);
        msg.coef_tq3y[i6 + j] = cMat(7, j);
        msg.coef_tq3z[i6 + j] = cMat(8, j);
        msg.coef_tq4x[i6 + j] = cMat(9, j);
        msg.coef_tq4y[i6 + j] = cMat(10, j);
        msg.coef_tq4z[i6 + j] = cMat(11, j);
      }
    }
  }
  
  void EGOReplanFSM::formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    bool success = false;
    swarm_central_pos_(0) = msg->pose.position.x;
    swarm_central_pos_(1) = msg->pose.position.y;
    swarm_central_pos_(2) = 1.5;
  
    end_pt_ = swarm_central_pos_;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),  //初始位置、速度、加速度
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()); //当前位置作为起始点，目标点作为终止点，不考虑避障等 生成全局轨迹

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** display ***/ 
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t).topRows(3);
      }

      end_vel_.setZero();
      have_target_ = true;   //若生成成功
      have_new_target_ = true;

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(bool flag_use_poly_init)
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur).topRows(3);
    start_vel_ = info->traj.getVel(t_cur).topRows(3);
    start_acc_ = info->traj.getAcc(t_cur).topRows(3);

    bool success = callReboundReplan(flag_use_poly_init);

    if (!success)
      return false;

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init)
  {
    
    //优化主程序
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_);  //start_pt_是当前无人机位置，end_pt_是地图上的鼠标给出的目标位置 planning_horizen_是待规划的距离=7.5m // 从当前位置向前规划7.5m 取其位置和速度作为目标

    Eigen::Matrix<double, 12, 1> desired_start_pt, desired_start_vel, desired_start_acc;
    double desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;  //考虑重新规划轨迹所需的时间
    if (have_local_traj_)
    {
      desired_start_pt =
          planner_manager_->traj_.local_traj.traj.getPos(desired_start_time - planner_manager_->traj_.local_traj.start_time);
      desired_start_vel =
          planner_manager_->traj_.local_traj.traj.getVel(desired_start_time - planner_manager_->traj_.local_traj.start_time);
      desired_start_acc =
          planner_manager_->traj_.local_traj.traj.getAcc(desired_start_time - planner_manager_->traj_.local_traj.start_time);
    }
    else
    {
      desired_start_pt << start_pt_, planner_manager_->cable_load_->init_Tq;
      desired_start_vel << start_vel_, Eigen::Matrix<double, 9, 1>::Zero();
      desired_start_acc << start_acc_, Eigen::Matrix<double, 9, 1>::Zero();
    }

    Eigen::Matrix<double, 12, 1> local_target_pt_sys, local_target_vel_sys;
    local_target_pt_sys << local_target_pt_, planner_manager_->cable_load_->init_Tq;
    local_target_vel_sys << local_target_vel_, Eigen::Matrix<double, 9, 1>::Zero();

    // local_target_pt_sys << end_pt_, planner_manager_->cable_load_->init_coef;
    // local_target_vel_sys << Eigen::Matrix<double, 9, 1>::Zero();

    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,
        desired_start_time, local_target_pt_sys, local_target_vel_sys,
        (have_new_target_ || flag_use_poly_init),  //若have_new_target = true 或上一次 计算local_traj 失败, 这时需要重新用Astar算法初始化
        have_local_traj_);

    have_new_target_ = false;

    if (plan_success)
    {
      traj_utils::PolyTraj msg;
      polyTraj2ROSMsg(msg);
      poly_traj_pub_.publish(msg); //若轨迹规划成功则发布消息 to odometry
      have_local_traj_ = true;
      cout << "load success" << endl;
    }
    return plan_success;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  
    {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "REPLAN_TRAJ", "EXEC_TRAJ", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }
} // namespace ego_planner
