#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include <traj_utils/Assignment.h>
#include "poly_traj_utils.hpp"
#include "munkres_algorithm.hpp"
#include <fstream>
#include <cable_load/cable_load.h>

#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;

namespace ego_planner
{

  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    
    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      points.resize(dim, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {
  
  private:
    GridMap::Ptr grid_map_;

    cable_load::Ptr cable_load_;
    
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    ConstrainPoints cps_;

    int cps_num_prePiece_; // number of distinctive constrain points each piece
    int variable_num_, variable_num_1, variable_num_2;     // optimization variables
    int piece_num_, piece_num_1, piece_num_2;        // poly traj piece numbers
    int iter_num_;         // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in swarm

    double collision_check_time_end_ = 0.0;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_swarm_;                       // swarm weight
    double wei_feas_;                        // feasibility weight
    double wei_sqrvar_;                      // squared variance weight
    double wei_time_;                        // time weight
    double weight_cable_length_;
    double weight_cable_colli_;
    double cable_tolerance_;
    double load_dist_;

    double obs_clearance_;                   // safe distance between uav and obstacles
    double swarm_clearance_;                 // safe distance between uav and uav
    double max_vel_, max_acc_;               // dynamic limits
    
    bool   is_other_assigning_ = false;

    double t_now_;

    double cable_length_;
    double uav_obs_clearance_, uav_swarm_clearance_, cable_clearance_;
    double weight_uav_obs_, weight_uav_swarm_, weight_FM_feasibility_, weight_FM_norm_, weight_cable_collision_;
    double wei_tq1_vel_, max_tq1_vel_;
    int cable_piece_;

    Eigen::MatrixXd c_output;
    Eigen::VectorXd T_output;
    double wei_energy_;

    int mem_size_, past_, max_iterations_, max_linesearch_;
    double g_epsilon_, delta_, min_step_, max_step_, f_dec_coeff_, s_curv_coeff_;

    Eigen::Matrix3d Transformb2e;

    double grav = 9.8;
    
    fstream result_file_;
  public:

    PolyTrajOptimizer() {
      time_t t;
      struct tm *tmp;
      time(&t);
      tmp = localtime(&t);
      char buf2[128];
      strftime(buf2, 64, "/home/tutu/our_lift12/cost_%Y-%m-%d %H:%M:%S.txt", tmp);
      result_file_.open(buf2, ios::app);
    };
    ~PolyTrajOptimizer() {
      result_file_.close();
    };

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const GridMap::Ptr &map, const cable_load::Ptr &CLoad);
    void setControlPoints(const Eigen::MatrixXd &points);

    /* helper functions */
    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr(void) { return &jerkOpt_; }
    inline poly_traj::MinJerkOpt getMinJerkOptPtr0() { return jerkOpt_; }

  
    
    inline int get_cps_num_prePiece_() { return cps_num_prePiece_; };
    double getCollisionCheckTimeEnd() { return collision_check_time_end_; }
  
    /* main planning API */
    bool OptimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, double trajectory_start_time);                                       
    void astarWithMinTraj( const Eigen::MatrixXd &iniState, 
                           const Eigen::MatrixXd &finState,
                           std::vector<Eigen::Vector3d> &simple_path,
                           Eigen::MatrixXd &ctl_points,
                           poly_traj::MinJerkOpt &frontendMJ);

    std::vector<Eigen::VectorXd> results_step;
    std::vector<std::vector<Eigen::VectorXd>> results_total;
  
  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback_forLoad(void *func_data, const double *x, double *grad, const int n);

    bool addFeasibilityForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1);
    
    bool addCollisionForUAV(const int i_dp, Eigen::VectorXd FMeach, std::vector<Eigen::Vector3d> uav_positions, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1, Eigen::VectorXd &grad2);
    
    bool addCollisionForCable(const int i_dp, Eigen::VectorXd FMeach, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1, Eigen::Vector3d pos_load, Eigen::VectorXd &grad2);

    bool addAverageForCableF(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1);
    bool addFeasibilitytq1vel(Eigen::Vector3d tq1_vel, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1);
   
    bool addSwarmForUAV(const int i_dp, Eigen::VectorXd FMeach, std::vector<Eigen::Vector3d> uav_positions, Eigen::VectorXd &grad0, double &cost, Eigen::VectorXd &grad1);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool obstacleGradCostP(const int i_dp,
                              const Eigen::Vector3d &p,
                              Eigen::Vector3d &gradp,
                              double &costp);            
    
    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);
    
    bool checkCollision(void);

    Eigen::Matrix3d DN(Eigen::Vector3d vec)
    {
      // return \partial{(vec/abs(vec))} / \partial vec
      Eigen::Vector3d q = vec/vec.norm();
      return (Eigen::Matrix3d::Identity()-q*q.transpose()) / vec.norm();
    };

  public:

    typedef unique_ptr<PolyTrajOptimizer> Ptr;

  };

} // namespace ego_planner
#endif