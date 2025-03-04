#pragma once

#include <robust_fast_navigation/rfn_types.h>

#include <Eigen/Core>
#include <vector>

class SolverBase
{
   public:
    virtual ~SolverBase() = default;

    virtual std::vector<rfn_state_t> get_trajectory() = 0;

    virtual bool setup(const Eigen::MatrixXd& start, const Eigen::MatrixXd& end,
                       const std::vector<Eigen::MatrixX4d>& polys) = 0;
    virtual bool solve()                                           = 0;

    // getters and setters
    virtual planner_params_t get_params() { return _params; }
    virtual double get_pos(double t, int dim)               = 0;
    virtual double get_vel(double t, int dim)               = 0;
    virtual void set_params(const planner_params_t& params) = 0;

   protected:
    planner_params_t _params;
};
