#include <robust_fast_navigation/JPS.h>
#include <robust_fast_navigation/planner_core.h>

#include <memory>

#ifdef CERES_FOUND
#include "robust_fast_navigation/contour_wrapper.h"
#endif
#include "robust_fast_navigation/faster_wrapper.h"
#include "robust_fast_navigation/gcopter_wrapper.h"
#include "robust_fast_navigation/solver_base.h"

Planner::Planner() {
  _is_map_set = false;
  _is_goal_set = false;
  _simplify_jps = false;
  _is_start_set = false;
  _plan_in_free = false;

  _trim_count = 0;

  _traj = {};

  _solver = std::make_unique<FasterWrapper>();
}

Planner::~Planner() {}

void Planner::set_params(const planner_params &params) {
  _params = params;

  if (_params.SOLVER == "faster")
    _solver = std::make_unique<FasterWrapper>();
  else if (_params.SOLVER == "gcopter")
    _solver = std::make_unique<GcopterWrapper>();
#ifdef CERES_FOUND
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

  bool start_occ = _map.is_occupied(_start(0, 0), _start(1, 0), "inflated");
  bool goal_occ = _map.is_occupied(_goal(0, 0), _goal(1, 0), "inflated");
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
  jps.set_util(_map, "inflated");

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

#if CERES_FOUND
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
  for (int i = 0; i < _traj.size(); ++i) {
    Eigen::Vector2d pos = _traj[i].pos.head(2);
    /*if (_map.is_occupied(pos[0], pos[1], "obstacles"))*/
    if (_map.get_cost(pos(0), pos(1), "obstacles")) {
      std::cout << termcolor::red
                << "[Planner Core] Trajectory overlaps obstacle"
                << termcolor::reset << std::endl;
      return PlannerStatus::TRAJ_GEN_FAIL;
    }
  }

  _is_start_set = false;
  _is_goal_set = false;
  _is_map_set = false;

  _old_goal = _goal;

  return PlannerStatus::SUCCESS;
}

std::vector<rfn_state_t> Planner::get_trajectory() {
  if (_traj.size() == 0)
    return {};

  std::vector<rfn_state_t> ret;

  int sz = _traj.size();
  // for (int i = 0; i < _traj.size(); ++i) {
  //   Eigen::Vector2d pos = _traj[i].pos.head(2);
  //   if (_map.is_occupied(pos[0], pos[1], "inflated") &&
  //       _map.get_signed_dist(pos[0], pos[1]) < _params.TRIM_DIST) {
  //     sz = i - 1;
  //     std::cout << termcolor::red
  //               << "[Planner Core] Trajectory overlaps obstacle, "
  //               << _map.get_signed_dist(pos[0], pos[1]) << " trimming to size
  //               "
  //               << sz << " / " << _traj.size() << termcolor::reset <<
  //               std::endl;
  //     if (_prev_plan_status)
  //       _trim_count++;
  //     else
  //       _trim_count = 0;
  //
  //     break;
  //   }
  // }

  // if (_trim_count > 10) {
  //   std::cout << termcolor::red
  //             << "[Planner Core] Trajectory trimmed too many times, "
  //             << "letting trajectory go" << termcolor::reset << std::endl;
  //   _trim_count = 0;
  //   return _traj;
  // }

  if (sz > 0)
    ret.insert(ret.end(), _traj.begin(), _traj.begin() + sz);
  else
    std::cout << termcolor::red << "[Planner Core] obs-free traj has 0 size!"
              << termcolor::reset << std::endl;

  return ret;
}

std::vector<rfn_state_t> Planner::get_arclen_traj() {
  std::vector<double> ss;
  std::vector<double> xs;
  std::vector<double> ys;

  std::vector<rfn_state_t> traj = get_trajectory();

  bool status = reparam_traj(traj, ss, xs, ys);

  if (!status)
    return {};

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

std::vector<rfn_state_t>
Planner::get_arclen_traj(const std::vector<rfn_state_t> &traj) {
  std::vector<double> ss;
  std::vector<double> xs;
  std::vector<double> ys;

  bool status = reparam_traj(traj, ss, xs, ys);

  if (!status)
    return {};

  std::vector<rfn_state_t> ret;
  ret.reserve(ss.size());

  for (int i = 0; i < ss.size(); ++i) {
    rfn_state_t &x = ret.emplace_back();
    x.pos(0) = xs[i];
    x.pos(1) = ys[i];
    x.t = ss[i];
  }

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

    _map.raycast(sx, sy, ex, ey, x, y, "inflated");

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
    _map.raycast(sx, sy, ex, ey, x, y, "inflated", &no_infos);

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

bool Planner::reparam_traj(const std::vector<rfn_state_t> &traj,
                           std::vector<double> &ss, std::vector<double> &xs,
                           std::vector<double> &ys) {
  /*_traj = _solver->get_trajectory();*/

  if (traj.size() == 0)
    return false;

  double traj_duration = traj.back().t;

  double total_length = compute_arclen(0, traj_duration);

  double M = 20;
  double ds = total_length / M;

  ss.resize(M + 1);
  xs.resize(M + 1);
  ys.resize(M + 1);

#ifdef CERES_FOUND
  if (_params.SOLVER == "contour") {
    // cast to contour solver
    /*ContourWrapper *contour_solver = dynamic_cast<ContourWrapper
     * *>(_solver.get());*/
    ds = traj_duration / M;
    for (int i = 0; i < M + 1; ++i) {
      ss[i] = i * ds;
      xs[i] = _solver->get_pos(ss[i], 0);
      ys[i] = _solver->get_pos(ss[i], 1);
    }

    return true;
  }
#endif

  double previous_ti = 0;
  for (int i = 0; i <= M; ++i) {
    double s = i * ds;

    double ti = binary_search(s, previous_ti, traj_duration, 1e-3);

    ss[i] = s;
    xs[i] = _solver->get_pos(ti, 0);
    ys[i] = _solver->get_pos(ti, 1);

    previous_ti = ti;
  }

  return true;
}

double Planner::binary_search(double dl, double start, double end,
                              double tolerance) {
  double t_left = start;
  double t_right = end;

  double prev_s = 0;
  double s = -1000;

  while (fabs(prev_s - s) > tolerance) {
    prev_s = s;

    double t_mid = (t_left + t_right) / 2;

    // always interested in total arc length up to t_mid
    s = compute_arclen(0, t_mid);

    // std::cout << "\ts at " << t_mid << " is " << s << std::endl;

    if (s < dl)
      t_left = t_mid;
    else
      t_right = t_mid;
  }

  return (t_left + t_right) / 2;
}

double Planner::compute_arclen(double t0, double tf) {
  // find arclength using trapezoid method
  double s = 0.0;
  double dt = (tf - t0) / 100.;

  double prev_dx = _solver->get_vel(t0, 0);
  double prev_dy = _solver->get_vel(t0, 1);

  for (double t = t0 + dt; t < tf; t += dt) {
    double dx, dy;
    dx = _solver->get_vel(t, 0);
    dy = _solver->get_vel(t, 1);

    // s += std::sqrt(dx * dx + dy * dy) * dt;
    s += .5 *
         (std::sqrt(dx * dx + dy * dy) +
          std::sqrt(prev_dx * prev_dx + prev_dy * prev_dy)) *
         dt;

    prev_dx = dx;
    prev_dy = dy;
  }

  return s;
}
