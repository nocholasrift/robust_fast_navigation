#pragma once

#include <Eigen/Core>

struct solver_state
{
    Eigen::Vector3d pos   = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel   = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d jerk  = Eigen::Vector3d::Zero();

    double t = 0;
};
typedef struct solver_state rfn_state_t;

struct planner_params
{
    std::string SOLVER;
    double V_MAX;
    double A_MAX;
    double J_MAX;
    double DT;
    double DT_FACTOR_INIT;
    double DT_FACTOR_FINAL;
    double DT_FACTOR_INCREMENT;
    double SOLVER_TRAJ_DT;
    double W_MAX;
    bool FORCE_FINAL_CONSTRAINT;
    int N_SEGMENTS;
    int N_THREADS;
    bool VERBOSE;
    bool USE_MINVO;
    bool PLAN_IN_FREE;
    bool SIMPLIFY_JPS;
    double MAX_SOLVE_TIME;
};
typedef struct planner_params planner_params_t;
