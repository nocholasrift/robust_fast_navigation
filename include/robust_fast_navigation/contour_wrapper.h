#pragma once

/*#include <robust_fast_navigation/contour_solver.h>*/
/*#include */
#include <robust_fast_navigation/map_util.h>
#include <robust_fast_navigation/ph_demo.h>
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
    void set_map(const map_util::occupancy_grid_t& map);

    double get_arclen();

   protected:
    PHSolver _solver;
};
