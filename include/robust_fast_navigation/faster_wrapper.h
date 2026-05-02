#pragma once

#include <robust_fast_navigation/solver_base.h>

#include <faster/solver.hpp>

class FasterWrapper : public SolverBase {
public:
  FasterWrapper();
  ~FasterWrapper();

  bool setup(const Eigen::MatrixXd &start, const Eigen::MatrixXd &end,
             const std::vector<Eigen::MatrixX4d> &polys) override;
  bool solve() override;

  std::vector<rfn_state_t> get_trajectory() override;
  RFNTrajectory get_rfn_trajectory() override;
  Eigen::MatrixXd get_ctrl_pts() override;

  double get_pos(double t, int dim) override;
  double get_vel(double t, int dim) override;
  void set_params(const planner_params_t &params) override;

  bool reparam_traj(std::vector<double> &ss, std::vector<double> &xs,
                    std::vector<double> &ys) override;

protected:
  faster::SolverGurobi _solver;
  RFNTrajectory _traj;
};
