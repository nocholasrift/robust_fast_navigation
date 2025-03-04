#pragma once

#include <robust_fast_navigation/solver_base.h>

#include <gcopter/gcopter.hpp>

class GcopterWrapper : public SolverBase
{
   public:
    GcopterWrapper();
    ~GcopterWrapper();
    bool setup(const Eigen::MatrixXd& start, const Eigen::MatrixXd& end,
               const std::vector<Eigen::MatrixX4d>& polys) override;
    bool solve() override;
    std::vector<rfn_state_t> get_trajectory() override;

    double get_pos(double t, int dim) override;
    double get_vel(double t, int dim) override;
    void set_params(const planner_params_t& params) override;

   protected:
    gcopter::GCOPTER_PolytopeSFC _solver;
    Trajectory<5> _traj;
};
