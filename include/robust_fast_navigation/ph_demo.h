#pragma once

#include <ceres/ceres.h>
#include <robust_fast_navigation/map_util.h>
#include <robust_fast_navigation/planar_ph.h>

#include <Eigen/Dense>
#include <cppad/ipopt/solve.hpp>
#include <map>
#include <memory>

class PHSolver
{
   public:
    PHSolver();
    void setStart(const Eigen::Vector2d &start);
    void setVel(const Eigen::Vector2d &vel);
    void setGoal(const Eigen::Vector2d &goal);
    void setSegments(size_t N);
    void setMap(const map_util::occupancy_grid_t &map);
    void setPolys(const std::vector<Eigen::MatrixX3d> &hpolys);
    const PlanarTrajectory<CppAD::AD<double>, CPPAD_TESTVECTOR(CppAD::AD<double>)> &getTraj()
        const;
    std::vector<double> warm_start();
    bool solve(const std::vector<double> &x0);

    std::vector<Eigen::VectorXd> x_;

   protected:
    size_t n_segments_;

    Eigen::Vector2d start_;
    Eigen::Vector2d vel_;
    Eigen::Vector2d goal_;

    map_util::occupancy_grid_t map_;

    bool constrain_vel_;

    std::vector<Eigen::MatrixX3d> hpolys_;
    std::unique_ptr<PlanarTrajectory<CppAD::AD<double>, CPPAD_TESTVECTOR(CppAD::AD<double>)>>
        ph_;
};
