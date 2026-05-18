#include "robust_fast_navigation/importance_sampler.h"
#include "robust_fast_navigation/map_util.h"
#include "robust_fast_navigation/rfn_types.h"
#include <robust_fast_navigation/JPS.h>
#include <robust_fast_navigation/planner_core.h>
#include <robust_fast_navigation/termcolor.hpp>

#include <memory>

#if defined(CERES_FOUND) && defined(GUROBI_FOUND)
#include "robust_fast_navigation/contour_wrapper.h"
#endif
#ifdef GUROBI_FOUND
#include "robust_fast_navigation/faster_wrapper.h"
#endif
// #include "robust_fast_navigation/faster_wrapper.h"

#include "robust_fast_navigation/gcopter_wrapper.h"
#include "robust_fast_navigation/solver_base.h"
#include "robust_fast_navigation/traj_util.h"

Planner::Planner() {
  _is_map_set = false;
  _is_goal_set = false;
  _simplify_jps = false;
  _is_start_set = false;
  _plan_in_free = false;

  _trim_count = 0;

  _traj = {};

  _solver = std::make_unique<GcopterWrapper>();
}

Planner::~Planner() {}

void Planner::set_params(const planner_params &params) {
  _params = params;

  if (_params.SOLVER == "gcopter")
    _solver = std::make_unique<GcopterWrapper>();
#ifdef GUROBI_FOUND
  else if (_params.SOLVER == "faster")
    _solver = std::make_unique<FasterWrapper>();
#endif
#if defined(CERES_FOUND) && defined(GUROBI_FOUND)
  else if (_params.SOLVER == "contour")
    _solver = std::make_unique<ContourWrapper>();
#endif
  else {
    std::cout << termcolor::red << "[Planner Core] Solver param value '"
              << _params.SOLVER << "' not recognized!" << termcolor::reset
              << std::endl;
    exit(-1);
  }

  _plan_in_free = params.PLAN_IN_FREE;
  _simplify_jps = params.SIMPLIFY_JPS;

  // solver params
  double w_max = params.W_MAX;
  double v_max = params.V_MAX;
  double a_max = params.A_MAX;
  double j_max = params.J_MAX;

  double limits[3] = {v_max, a_max, j_max};
  double factor_init = params.DT_FACTOR_INIT;
  double factor_final = params.DT_FACTOR_FINAL;
  double factor_increment = params.DT_FACTOR_INCREMENT;

  _solver->set_params(params);
}

void Planner::set_start(const Eigen::MatrixXd &start) {
  _start = start;
  _is_start_set = true;
}

// void set_start_from_traj(
//   const Eigen::Vector3d& current_pose,
//   std::shared_ptr<const Trajectory> traj,
//   double duration_since_start)
// {
//   // _traj_dt =
//   // Eigen::Vector3d
// }

void Planner::set_goal(const Eigen::MatrixXd &goal) {
  _goal = goal;
  if (!_is_start_set) {
    std::cerr << "[Planner Core] start must be set before goal!\n";
    return;
  }
  std::vector<double> clamped_goal = _map.clamp_point_to_bounds(
      {_start(0, 0), _start(1, 0)}, {goal(0, 0), goal(1, 0)});
  _goal(0, 0) = clamped_goal[0];
  _goal(1, 0) = clamped_goal[1];

  _is_goal_set = true;
}

void Planner::set_costmap(const map_util::occupancy_grid_t &map) {
  _map = map;
  // std::vector<double> origin = _map.get_origin();
  // std::cout << "origin: " << origin[0] << " " << origin[1] << "\n";
  _is_map_set = true;
}

PlannerStatus Planner::plan(double horizon,
                            std::vector<Eigen::Vector2d> &jpsPath,
                            std::vector<Eigen::MatrixX4d> &hPolys) {
  if (!_is_start_set || !_is_goal_set || !_is_map_set) {
    std::cout << termcolor::red
              << "[Planner Core] missing start, goal or costmap"
              << termcolor::reset << std::endl;
    return PlannerStatus::MISC_FAILURE;
  }

  _prev_plan_status = false;

  /*_sdf_solver = std::make_unique<BezierSdfNLP>(*/
  /*    _params.N_SEGMENTS, 1, [this](double x, double y) { return
   * _map->get_dist(x, y); });*/

  /*************************************
  ************ PERFORM  JPS ************
  **************************************/

  bool start_occ =
      _map.is_occupied(_start(0, 0), _start(1, 0), map_util::Layer::kInflated);
  bool goal_occ =
      _map.is_occupied(_goal(0, 0), _goal(1, 0), map_util::Layer::kInflated);
  /*bool start_occ =*/
  /*    _map.get_signed_dist(_start(0, 0), _start(1, 0)) < 2 *
   * _map.get_resolution();*/
  /*bool goal_occ =*/
  /*    _map.get_signed_dist(_start(0, 0), _start(1, 0)) < 2 *
   * _map.get_resolution();*/
  // ros::Time start_frontend = ros::Time::now();

  jps::JPSPlan jps;
  unsigned int sX, sY, eX, eY;

  // potentially change this to odom instead of start
  std::vector<unsigned int> start_cells =
      _map.world_to_map(_start(0, 0), _start(1, 0));
  sX = start_cells[0];
  sY = start_cells[1];
  // std::cout << "startind is " << _map.cells_to_index(sX, sY) << std::endl;
  // std::cout << "sX = " << sX << std::endl;
  // std::cout << "sY = " << sY << std::endl;

  std::vector<unsigned int> goal_cells =
      _map.world_to_map(_goal(0, 0), _goal(1, 0));
  eX = goal_cells[0];
  eY = goal_cells[1];

  jps.set_start(sX, sY);
  jps.set_destination(eX, eY);

  double x = _map.get_origin()[0];
  double y = _map.get_origin()[1];
  double resolution = _map.get_resolution();
  int w = _map.get_size()[0];
  int h = _map.get_size()[1];

  /*jps.set_map(_map.get_data("inflated"), w, h, x, y, resolution);*/
  jps.set_util(_map, map_util::Layer::kInflated);

  // ros::Time start_jps = ros::Time::now();
  int jps_status = jps.JPS();
  // std::cout << "[Planner Core] JPS took "
  //           << (ros::Time::now() - start_jps).toSec() << "s" << std::endl;

  // try one more time without inflated obstacles...
  /*if (jps_status == IN_OCCUPIED_SPACE)*/
  /*{*/
  /*    std::cout << termcolor::red << "[Planner Core] JPS failed, trying again
   * with "*/
  /*              << "LETHAL_OBSTACLE" << termcolor::reset << std::endl;*/
  /*    jps.set_occ_value(costmap_2d::LETHAL_OBSTACLE);*/
  /*    jps.set_map(_map.get_data("obstacles"), w, h, x, y, resolution);*/
  /*    jps_status = jps.JPS();*/
  /*}*/

  std::vector<Eigen::Vector2d> oldJps = jpsPath;

  jpsPath = jps.getPath(_simplify_jps);

  if (jpsPath.size() < 2) {
    std::cout << termcolor::red << "[Planner Core] JPS failed"
              << termcolor::reset << std::endl;
    if (jps_status == IN_OCCUPIED_SPACE) {
      std::cout << termcolor::red
                << "[Planner Core] JPS failed, start in occupied space"
                << termcolor::reset << std::endl;

      return PlannerStatus::START_IN_OBSTACLE;
    } else {
      std::cout << termcolor::red << "[Planner Core] JPS failed to find path"
                << termcolor::reset << std::endl;
      return PlannerStatus::JPS_FAIL_NO_PATH;
    }
  }

  // check if old jps path has value
  if (oldJps.size() > 0 && !JPSIntersectObs(oldJps)) {
    // if new path angle is much different from old path with nearly same
    // length, use old path
    Eigen::Vector2d oldDir = oldJps[oldJps.size() - 1] - oldJps[0];
    Eigen::Vector2d newDir = jpsPath[jpsPath.size() - 1] - jpsPath[0];

    oldDir = oldDir / oldDir.norm();
    newDir = newDir / newDir.norm();

    double angle = acos(oldDir.dot(newDir)) * 180 / M_PI;
    std::cout << termcolor::on_magenta
              << "Old jps given, and doesn't intersect obs" << std::endl;
    std::cout << "angle between old and new path is " << angle
              << termcolor::reset << std::endl;

    if (angle > 30) {
      // if new path is longer or barely shorter than old path, use old
      // path
      double oldLen = 0;
      for (int i = 0; i < oldJps.size() - 1; ++i) {
        oldLen += (oldJps[i + 1] - oldJps[i]).norm();
      }

      double newLen = 0;
      for (int i = 0; i < jpsPath.size() - 1; ++i) {
        newLen += (jpsPath[i + 1] - jpsPath[i]).norm();
      }

      if (newLen > .98 * oldLen) {
        jpsPath = oldJps;
      }
      std::cout << termcolor::on_magenta << "new len is " << newLen
                << "\nand old len is " << oldLen << termcolor::reset
                << std::endl;
    }
  }

  if (jpsPath.size() > _params.MAX_POLYS + 1)
    jpsPath.erase(jpsPath.begin() + _params.MAX_POLYS + 1, jpsPath.end());

  /*************************************
  ************* REFINE JPS *************
  **************************************/

  std::vector<Eigen::Vector2d> newJPSPath;

  if (_plan_in_free) {
    jpsPath = getJPSInFree(jpsPath);
  }

  double jps_path_length = 0;
  for (int i = 0; i < jpsPath.size() - 1; ++i) {
    jps_path_length += (jpsPath[i + 1] - jpsPath[i]).norm();
  }

  std::cout << "JPS path lenth is " << jps_path_length << " / " << horizon
            << std::endl;
  if (jps.truncateJPS(jpsPath, newJPSPath, horizon)) {
    jpsPath = newJPSPath;
  }

  _goal << Eigen::Vector3d(jpsPath.back()[0], jpsPath.back()[1], 0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

  if (jpsPath.size() == 0) {
    std::cout << termcolor::red << "[Planner Core] JPS modifications failed"
              << termcolor::reset << std::endl;
    return PlannerStatus::JPS_FAIL_NO_PATH;
  }

  /*************************************
  ********* GENERATE POLYTOPES *********
  **************************************/

  std::cout << "generating polytopes\n";
  hPolys.clear();
  // ros::Time before_corridor = ros::Time::now();
  if (!corridor::createCorridorJPS(jpsPath, _map, hPolys, _start, _goal)) {
    std::cout << termcolor::red << "[Planner Core] Corridor creation failed"
              << termcolor::reset << std::endl;
    return PlannerStatus::CORRIDOR_FAIL;
  }
  std::cout << "done\n";

  // if (!corridor::isInPoly(hPolys.back(), Eigen::Vector2d(_goal(0, 0),
  // _goal(1, 0))) &&
  //     jpsPath.size() > 1)
  // {
  //     // iterate over points in last segment of JPS path to find point in
  //     corridor Eigen::Vector2d segment = jpsPath.back() -
  //     jpsPath[jpsPath.size() - 2]; double segment_length   = segment.norm();
  //     double step_size        = segment_length / 10;

  //     segment.normalize();

  //     for (int i = 0; i < 10; ++i)
  //     {
  //         Eigen::Vector2d point = jpsPath[jpsPath.size() - 2] + segment *
  //         step_size * i; if (corridor::isInPoly(hPolys.back(), point))
  //         {
  //             _goal(0, 0) = point(0);
  //             _goal(1, 0) = point(1);
  //             break;
  //         }
  //     }
  // }

  bool is_in_corridor = false;

  // if adjacent polytopes don't overlap, don't plan
  for (int p = 0; p < hPolys.size() - 1; p++) {
    if (!geo_utils::overlap(hPolys[p], hPolys[p + 1])) {
      // ROS_ERROR("CORRIDOR IS NOT FULLY CONNECTED");
      return PlannerStatus::CORRIDOR_FAIL;
    }

    if (!is_in_corridor)
      is_in_corridor = corridor::isInPoly(
          hPolys[p], Eigen::Vector2d(_start(0, 0), _start(1, 0)));
  }
  // std::cout << "[Planner Core] Corridor creation took "
  //           << (ros::Time::now() - before_corridor).toSec() << " seconds"
  //           << std::endl;
  // if(corridor::union_corridor(hPolys, _cgal_border, _corridor_boundary))
  //     std::cout << "we did it!!!" << std::endl;

  // std::cout << "[Planner Core] Front end took "
  //           << (ros::Time::now() - start_frontend).toSec() << " seconds"
  //           << std::endl;
  /*************************************
  ******** GENERATE  TRAJECTORY ********
  **************************************/

  if (!_solver->setup(_start, _goal, hPolys)) {
    std::cout << termcolor::red << "[Planner Core] Solver setup failed"
              << termcolor::reset << std::endl;
    return PlannerStatus::TRAJ_GEN_FAIL;
  }

#if defined(CERES_FOUND) && defined(GUROBI_FOUND)
  if (_params.SOLVER == "contour") {
    ContourWrapper *contour_solver =
        dynamic_cast<ContourWrapper *>(_solver.get());
    contour_solver->set_map(_map);
  }
#endif

  // time trajectory generation
  // ros::Time start_solve = ros::Time::now();
  if (!_solver->solve()) {
    std::cout << termcolor::red << "[Planner Core] Generating trajectory failed"
              << termcolor::reset << std::endl;
    return PlannerStatus::TRAJ_GEN_FAIL;
  } else {
    std::cout << termcolor::green << "[Planner Core] Solver found trajectory"
              << termcolor::reset << std::endl;
    _prev_plan_status = true;
  }

  _traj = _solver->get_trajectory();

  // ensure trajectory does not overlap lethal obstacles
  // for (int i = 0; i < _traj.size(); ++i) {
  //   Eigen::Vector2d pos = _traj[i].pos.head(2);
  //   /*if (_map.is_occupied(pos[0], pos[1], "obstacles"))*/
  //   if (_map.get_cost(pos(0), pos(1), map_util::Layer::kObstacles)) {
  //     std::cout << termcolor::red
  //               << "[Planner Core] Trajectory overlaps obstacle"
  //               << termcolor::reset << std::endl;
  //     return PlannerStatus::TRAJ_GEN_FAIL;
  //   }
  // }

  _is_start_set = false;
  _is_goal_set = false;
  _is_map_set = false;

  _old_goal = _goal;

  return PlannerStatus::SUCCESS;
}

Eigen::VectorXf pack_free(const Eigen::MatrixXf &all_cps, int n_segments) {
  // free vars: CP2_s0, CP3_s0..CP3_s{N-2}
  // = 2 + (n_segments - 1) points = n_segments + 1 points
  int n_free = (n_segments - 1) * 2;
  Eigen::VectorXf free(n_free);

  // CP1_s0 is fixed since velocity vector should stay the same!

  // free.segment<2>(0) = all_cps.row(2); // CP2_s0

  for (int seg = 0; seg < n_segments - 1; ++seg)
    free.segment<2>(seg * 2) = all_cps.row(seg * 4 + 3); // CP3_seg

  return free;
}

// Reconstruct all CPs from free variables
Eigen::MatrixXf unpack_free(const Eigen::VectorXf &v,
                            const Eigen::Vector2f &start,
                            const Eigen::Vector2f &cp1_s0,
                            const Eigen::Vector2f &cp2_s0,
                            const Eigen::Vector2f &goal, int n_segments) {
  Eigen::MatrixXf cps(n_segments * 4, 2);

  // Segment 0 - fully specified
  cps.row(0) = start;
  cps.row(1) = cp1_s0;
  cps.row(2) = cp2_s0;
  cps.row(3) = v.segment<2>(0);

  // C1 / C2 constraints allow derivation of most other cps
  for (int seg = 1; seg < n_segments; ++seg) {
    Eigen::Vector2f prev_cp1 = cps.row(seg * 4 - 3);
    Eigen::Vector2f prev_cp2 = cps.row(seg * 4 - 2);
    Eigen::Vector2f prev_cp3 = cps.row(seg * 4 - 1);

    Eigen::Vector2f new_cp0 = prev_cp3;                   // C0
    Eigen::Vector2f new_cp1 = 2.0f * prev_cp3 - prev_cp2; // C1
    Eigen::Vector2f new_cp2 =
        4.0f * prev_cp3 - 4.0f * prev_cp2 + prev_cp1; // C2
    Eigen::Vector2f new_cp3 =
        (seg < n_segments - 1) ? v.segment<2>(seg * 2) : goal;

    cps.row(seg * 4 + 0) = new_cp0;
    cps.row(seg * 4 + 1) = new_cp1;
    cps.row(seg * 4 + 2) = new_cp2;
    cps.row(seg * 4 + 3) = new_cp3;
  }
  return cps;
}

Eigen::VectorXf build_std_dev(float base_std, int n_segments) {
  Eigen::VectorXf std_dev((n_segments - 1) * 2);

  // CP2_s0 - only propagates through C2 formula once, so scale by 4^(N-1)
  // std_dev.segment<2>(0) =
  //     Eigen::Vector2f::Constant(base_std / std::pow(4.0f, n_segments - 1));

  // CP3 of each segment
  for (int seg = 0; seg < n_segments - 1; ++seg) {
    float scale = std::pow(4.0f, (n_segments - 2 - seg));
    std_dev.segment<2>(seg * 2) = Eigen::Vector2f::Constant(base_std / scale);
  }

  return std_dev;
}

RFNTrajectory Planner::refine_traj() {
  // Importance Sampling to push away from obstacles
  Eigen::MatrixXf cps = _solver->get_ctrl_pts().cast<float>();
  int n_segments = _params.N_SEGMENTS;

  Eigen::VectorXf free_cps = pack_free(cps, n_segments);

  Eigen::Vector2f start = cps.row(0);
  Eigen::Vector2f cp1_s0 = cps.row(1);
  Eigen::Vector2f cp2_s0 = cps.row(2);
  // if ((start - cp1_s0).squaredNorm() < 1e-3) {
  //   cp1_s0 = start + Eigen::Vector2f(0.01, 0.0);
  // }

  Eigen::Vector2f goal = cps.row(cps.rows() - 1);
  auto cost_fn = [&, this](const Eigen::VectorXf &sample) {
    Eigen::MatrixXf cp_sample =
        unpack_free(sample, start, cp1_s0, cp2_s0, goal, n_segments);
    return this->sampling_cost(cp_sample, cps);
  };

  std::cout << "----- COST ITER -1 -----\n";
  sampling_cost(cps, cps, true);

  float base_std = 2e-1;
  Eigen::VectorXf std_vec = build_std_dev(base_std, n_segments);
  ImportanceSampler<decltype(cost_fn), float> sampler(cost_fn, std_vec, 1.0, 20,
                                                      100);
  Eigen::VectorXf optimized_cps = sampler.optimize(free_cps);
  Eigen::MatrixXf new_cps =
      unpack_free(optimized_cps, start, cp1_s0, cp2_s0, goal, n_segments);

  std::cout << "----- COST ITER FINAL -----\n";
  sampling_cost(new_cps, cps, true);

  Eigen::Vector2f pre_vel = (3 * (cps.row(1) - cps.row(0)));
  Eigen::Vector2f post_vel = (3 * (new_cps.row(1) - new_cps.row(0)));
  if (pre_vel.norm() > 1e-3) {
    std::cout << "velocity before: " << pre_vel.transpose() / pre_vel.norm()
              << "\n";
    std::cout << "velocity after: " << post_vel.transpose() / post_vel.norm()
              << "\n";
  }

  double segment_duration = 1.0;
  RFNTrajectory traj(new_cps.cast<double>(), segment_duration);

  return traj;
}

std::vector<rfn_state_t> Planner::get_trajectory() {
  if (_traj.size() == 0)
    return {};

  std::vector<rfn_state_t> ret;

  int sz = _traj.size();

  if (sz > 0)
    ret.insert(ret.end(), _traj.begin(), _traj.begin() + sz);
  else
    std::cout << termcolor::red << "[Planner Core] obs-free traj has 0 size!"
              << termcolor::reset << std::endl;

  return ret;
}

std::vector<rfn_state_t> Planner::get_arclen_traj(bool refine) {
  std::vector<double> ss;
  std::vector<double> xs;
  std::vector<double> ys;

  // std::vector<rfn_state_t> traj = get_trajectory();

  // RFNTrajectory traj;
  // if (refine) {
  //   traj = refine_traj();
  // } else {
  //   traj = _solver->get_rfn_trajectory();
  // }
  //
  // bool status = traj_utils::reparam_traj(traj, ss, xs, ys);
  bool status = _solver->reparam_traj(ss, xs, ys);

  if (!status) {
    std::cout << termcolor::yellow
              << "[Planner] Warning: reparam traj status returned false!"
              << termcolor::reset << std::endl;
    return {};
  }

  std::vector<rfn_state_t> ret;
  ret.reserve(ss.size());

  for (int i = 0; i < ss.size(); ++i) {
    rfn_state_t &x = ret.emplace_back();
    x.pos(0) = xs[i];
    x.pos(1) = ys[i];
    x.t = ss[i];
  }

  /*_map.push_trajectory(ret);*/

  return ret;
}

std::vector<Eigen::Vector3d> Planner::get_cps() {
  std::vector<Eigen::Vector3d> ret;

  return ret;
}

bool Planner::JPSIntersectObs(const std::vector<Eigen::Vector2d> &path) {
  if (!_is_map_set) {
    std::cout << termcolor::yellow << "[Planner] costmap not yet set"
              << termcolor::reset << std::endl;
    return false;
  }

  for (int i = 0; i < path.size() - 1; i++) {
    double x, y;
    unsigned int sx, sy, ex, ey;

    std::vector<unsigned int> start_cells =
        _map.world_to_map(path[i][0], path[i][1]);
    sx = start_cells[0];
    sy = start_cells[1];

    std::vector<unsigned int> end_cells =
        _map.world_to_map(path[i + 1][0], path[i + 1][1]);
    ex = end_cells[0];
    ey = end_cells[1];

    _map.raycast(sx, sy, ex, ey, x, y, map_util::Layer::kObstacles);

    // if x,y does not reach the end of the ray, we hit an unknown cell
    Eigen::Vector2d p(x, y);

    if ((path[i + 1] - p).norm() > _map.get_resolution()) {
      std::vector<unsigned int> p_cells = _map.world_to_map(p[0], p[1]);
      return true;
    }
  }

  return false;
}

std::vector<Eigen::Vector2d>
Planner::getJPSInFree(const std::vector<Eigen::Vector2d> &path) {
  if (!_is_map_set) {
    std::cout << termcolor::yellow << "[Planner] costmap not yet set"
              << termcolor::reset << std::endl;
    return path;
  }

  std::vector<Eigen::Vector2d> ret;

  if (path.size() < 2) {
    std::cout << termcolor::yellow << "[Planner] JPS only contains one point"
              << termcolor::reset << std::endl;
    return path;
  }

  // walk along jps and find first unknown cell

  double x, y;
  unsigned int sx, sy, ex, ey;

  ret.push_back(path[0]);

  // raycast along JPS until we hit an unknown cell
  for (int i = 0; i < path.size() - 1; i++) {
    std::vector<unsigned int> start_cells =
        _map.world_to_map(path[i][0], path[i][1]);
    sx = start_cells[0];
    sy = start_cells[1];

    std::vector<unsigned int> end_cells =
        _map.world_to_map(path[i + 1][0], path[i + 1][1]);
    ex = end_cells[0];
    ey = end_cells[1];

    std::vector<unsigned char> no_infos = _map.get_no_info_values();
    _map.raycast(sx, sy, ex, ey, x, y, map_util::Layer::kInflated, &no_infos);

    // if x,y does not reach the end of the ray, we hit an unknown cell
    Eigen::Vector2d p(x, y);

    if ((path[i + 1] - p).norm() > 1e-3) {
      ret.push_back(p);
      break;
    }

    ret.push_back(path[i + 1]);
  }

  return ret;
}

float Planner::sampling_cost(const Eigen::MatrixXf &cps,
                             const Eigen::MatrixXf &orig_cps, bool verbose) {
  // std::cout << "cp_sample: \n" << cps << "\n";
  // std::cout << "original_cp: \n" << orig_cps << "\n";
  int n_segments = _params.N_SEGMENTS;
  int N_points = 20;

  float jerk_cost = 0.0;
  float reg_cost = 0.0;
  float obstacle_cost = 0.0;
  float cost = 0.0;

  for (int seg = 0; seg < n_segments; ++seg) {
    Eigen::Vector2f p0 = cps.row(seg * 4 + 0);
    Eigen::Vector2f p1 = cps.row(seg * 4 + 1);
    Eigen::Vector2f p2 = cps.row(seg * 4 + 2);
    Eigen::Vector2f p3 = cps.row(seg * 4 + 3);

    // obstacle avoidance cost
    for (int i = 0; i < N_points; ++i) {
      float tau = static_cast<float>(i) / (N_points - 1);
      Eigen::Vector2f point = evaluate_bezier_segment(p0, p1, p2, p3, tau);
      float sdf_val =
          _map.sdf_dist(point[0], point[1], map_util::Layer::kInflated);
      float diff = std::max(0.0f, 0.1f - sdf_val);
      obstacle_cost += diff * diff;
    }

    // jerk cost
    Eigen::Vector2f jerk = p3 - 3.0f * p2 + 3.0f * p1 - p0;
    jerk_cost += jerk.squaredNorm();
  }

  // regularization in full CP space
  Eigen::VectorXf cps_flat =
      Eigen::Map<const Eigen::VectorXf>(cps.data(), cps.size());
  Eigen::VectorXf orig_flat =
      Eigen::Map<const Eigen::VectorXf>(orig_cps.data(), orig_cps.size());

  reg_cost += (cps_flat - orig_flat).squaredNorm();

  float obs_lambda = 5.0f;
  float jerk_lambda = 0.2f;
  float reg_lambda = 1.0f;
  cost = obs_lambda * obstacle_cost + jerk_lambda * jerk_cost +
         reg_lambda * reg_cost;

  if (verbose) {
    std::cout << "\t* obs cost: " << obs_lambda * obstacle_cost << "\n";
    std::cout << "\t* jerk cost: " << jerk_lambda * jerk_cost << "\n";
    std::cout << "\t* reg cost: " << reg_lambda * reg_cost << "\n";
  }

  return cost;
}

Eigen::Vector2f Planner::evaluate_bezier_segment(const Eigen::Vector2f &p0,
                                                 const Eigen::Vector2f &p1,
                                                 const Eigen::Vector2f &p2,
                                                 const Eigen::Vector2f &p3,
                                                 float t) {
  float mt = 1.0 - t;
  return mt * mt * mt * p0 + 3.0 * mt * mt * t * p1 + 3.0 * mt * t * t * p2 +
         t * t * t * p3;
}
