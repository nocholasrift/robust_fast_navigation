#pragma once

/*#include <robust_fast_navigation/contour_solver.h>*/
/*#include */
#include <robust_fast_navigation/planar_ph.h>
#include <robust_fast_navigation/solver_base.h>

class ContourWrapper : public SolverBase
{
   public:
    ContourWrapper();
    ~ContourWrapper();
    bool setup(const Eigen::MatrixXd& start, const Eigen::MatrixXd& end,
               const std::vector<Eigen::MatrixX4d>& polys) override;
    bool solve() override;
    std::vector<rfn_state_t> get_trajectory() override;

    double get_pos(double t, int dim) override;
    double get_vel(double t, int dim) override;
    void set_params(const planner_params_t& params) override;

   protected:
    planar_ph::PHContourSolver _solver;
};
