#pragma once

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <robust_fast_navigation/contour_solver.h>
#include <robust_fast_navigation/corridor.h>
#include <robust_fast_navigation/faster_wrapper.h>
#include <robust_fast_navigation/map_util.h>
#include <robust_fast_navigation/spline.h>

class SplineWrapper
{
   public:
    tk::spline spline;  // Expose the tk::spline
};
typedef SplineWrapper spline_t;

enum PlannerStatus
{
    SUCCESS           = 0,
    MISC_FAILURE      = 1,
    JPS_FAIL_NO_PATH  = 2,
    START_IN_OBSTACLE = 3,
    CORRIDOR_FAIL     = 4,
    TRAJ_GEN_FAIL     = 5,
};

class Planner
{
   public:
    Planner();
    ~Planner();

    void set_params(const planner_params_t &params);

    // setters

    void set_start(const Eigen::MatrixXd &start);
    void set_goal(const Eigen::MatrixXd &goal);
    void set_costmap(const map_util::occupancy_grid_t &map);

    // #ifdef FOUND_PYBIND11
    // void set_start(const Eigen::MatrixXd& start);
    // void set_goal(const Eigen::MatrixXd& goal);
    // void set_costmap(const std::shared_ptr<costmap_2d::Costmap2D>& map);
    // #endif

    // getters
    std::vector<Eigen::Vector3d> get_cps();
    std::vector<rfn_state_t> get_trajectory();
    std::vector<rfn_state_t> get_arclen_traj();

    // std::vector<spline_t> get_tube();

    // Eigen::Matrix3Xd get_corridor_boundary();

    PlannerStatus plan(double horizon, std::vector<Eigen::Vector2d> &jpsPath,
                       std::vector<Eigen::MatrixX4d> &hPolys);

    std::vector<Eigen::Vector2d> getJPSInFree(const std::vector<Eigen::Vector2d> &path);
    bool JPSIntersectObs(const std::vector<Eigen::Vector2d> &path);

   private:
    Eigen::MatrixXd _old_goal;
    Eigen::MatrixXd _start;
    Eigen::MatrixXd _goal;
    Eigen::Matrix3Xd _corridor_boundary;

    // Polygon_2 _cgal_border;

    bool _is_occ;
    bool _is_map_set;
    bool _is_goal_set;
    bool _simplify_jps;
    bool _plan_in_free;
    bool _is_start_set;

    std::unique_ptr<SolverBase> _solver;
    std::vector<rfn_state_t> _traj;

    planner_params_t _params;

    map_util::occupancy_grid_t _map;

    double binary_search(double dl, double start, double end, double tolerance);
    double compute_arclen(double t0, double tf);
    bool reparam_traj(std::vector<double> &ss, std::vector<double> &xs,
                      std::vector<double> &ys);
};
