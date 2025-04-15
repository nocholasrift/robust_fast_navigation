#pragma once

#include <Eigen/Core>

#include "ros/topic.h"

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

class Segment
{
   public:
    Segment()  = default;
    ~Segment() = default;

    Eigen::VectorXd getPos(double t)
    {
        return _x[0] * t * t * t + _x[1] * t * t + _x[2] * t + _x[3];
    }
    Eigen::VectorXd getVel(double t) { return 3 * _x[0] * t * t + 2 * _x[1] * t + _x[2]; }

    Eigen::VectorXd getAcc(double t) { return 6 * _x[0] * t + 2 * _x[1]; }

    Eigen::VectorXd getJerk(double t) { return 6 * _x[0]; }

    Eigen::VectorXd getP0() { return _x[3]; }

    Eigen::VectorXd getP1() { return (_x[2] * _duration + 3 * _x[3]) / 3; }

    Eigen::VectorXd getP2()
    {
        return (_x[1] * _duration * _duration + 2 * _x[2] * _duration + 3 * _x[3]) / 3;
    }

    Eigen::VectorXd getP3() { return getPos(_duration); }

    std::array<Eigen::VectorXd, 4> _x;
    double _duration;
};

class RFNTrajectory
{
   public:
    RFNTrajectory() = default;

    Eigen::VectorXd getPos(double t)
    {
        int m          = getSegment(t);
        double start_t = getSegmentStartTime(m);
        return _segments[m].getPos(t - start_t);
    }

    Eigen::VectorXd getVel(double t)
    {
        int m          = getSegment(t);
        double start_t = getSegmentStartTime(m);
        return _segments[m].getVel(t - start_t);
    }

    Eigen::VectorXd getAcc(double t)
    {
        int m          = getSegment(t);
        double start_t = getSegmentStartTime(m);
        return _segments[m].getAcc(t - start_t);
    }

    Eigen::VectorXd getJerk(double t)
    {
        int m          = getSegment(t);
        double start_t = getSegmentStartTime(m);
        return _segments[m].getJerk(t - start_t);
    }

    int getSegment(double t)
    {
        for (int i = _segments.size() - 1; i >= 0; --i)
        {
            if (t >= getSegmentStartTime(i)) return i;
        }

        return _segments.size() - 1;
    }

    double getSegmentStartTime(int m)
    {
        double t = 0.;
        for (int i = 0; i < m; ++i)
        {
            t += _segments[i]._duration;
        }

        return t;
    }

    std::vector<Segment> _segments;
};
