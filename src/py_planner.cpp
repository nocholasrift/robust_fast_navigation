#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <robust_fast_navigation/map_util.h>
#include <robust_fast_navigation/planner_core.h>
#include <robust_fast_navigation/rfn_types.h>

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector2d>);
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::MatrixX4d>);

PYBIND11_MODULE(py_planner, m) {
  py::bind_vector<std::vector<Eigen::Vector2d>>(m, "vec_Vec2d");
  py::bind_vector<std::vector<Eigen::MatrixX4d>>(m, "vec_MatX4d");

  py::enum_<PlannerStatus>(m, "PlannerStatus")
      .value("SUCCESS", PlannerStatus::SUCCESS)
      .value("MISC_FAILURE", PlannerStatus::MISC_FAILURE)
      .value("JPS_FAIL_NO_PATH", PlannerStatus::JPS_FAIL_NO_PATH)
      .value("START_IN_OBSTACLE", PlannerStatus::START_IN_OBSTACLE)
      .value("CORRIDOR_FAIL", PlannerStatus::CORRIDOR_FAIL)
      .value("TRAJ_GEN_FAIL", PlannerStatus::TRAJ_GEN_FAIL);

  py::class_<solver_state>(m, "RFNState")
      .def_readwrite("pos", &solver_state::pos)
      .def_readwrite("vel", &solver_state::vel)
      .def_readwrite("accel", &solver_state::accel)
      .def_readwrite("jerk", &solver_state::jerk)
      .def_readwrite("t", &solver_state::t);

  py::class_<planner_params>(m, "PlannerParams")
      .def(py::init<>())
      .def_readwrite("SOLVER", &planner_params::SOLVER)
      .def_readwrite("V_MAX", &planner_params::V_MAX)
      .def_readwrite("A_MAX", &planner_params::A_MAX)
      .def_readwrite("J_MAX", &planner_params::J_MAX)
      .def_readwrite("DT", &planner_params::DT)
      .def_readwrite("DT_FACTOR_INIT", &planner_params::DT_FACTOR_INIT)
      .def_readwrite("DT_FACTOR_FINAL", &planner_params::DT_FACTOR_FINAL)
      .def_readwrite("DT_FACTOR_INCREMENT",
                     &planner_params::DT_FACTOR_INCREMENT)
      .def_readwrite("SOLVER_TRAJ_DT", &planner_params::SOLVER_TRAJ_DT)
      .def_readwrite("W_MAX", &planner_params::W_MAX)
      .def_readwrite("FORCE_FINAL_CONSTRAINT",
                     &planner_params::FORCE_FINAL_CONSTRAINT)
      .def_readwrite("N_SEGMENTS", &planner_params::N_SEGMENTS)
      .def_readwrite("N_THREADS", &planner_params::N_THREADS)
      .def_readwrite("VERBOSE", &planner_params::VERBOSE)
      .def_readwrite("USE_MINVO", &planner_params::USE_MINVO)
      .def_readwrite("PLAN_IN_FREE", &planner_params::PLAN_IN_FREE)
      .def_readwrite("SIMPLIFY_JPS", &planner_params::SIMPLIFY_JPS)
      .def_readwrite("MAX_SOLVE_TIME", &planner_params::MAX_SOLVE_TIME)
      .def_readwrite("TRIM_DIST", &planner_params::TRIM_DIST)
      .def_readwrite("MAX_POLYS", &planner_params::MAX_POLYS);

  py::class_<Planner>(m, "Planner")
      .def(py::init<>())
      .def("set_params", &Planner::set_params)
      .def("set_start", &Planner::set_start)
      .def("set_goal", &Planner::set_goal)
      .def("set_costmap", &Planner::set_costmap)
      .def("plan", &Planner::plan)
      .def("get_arclen_traj", py::overload_cast<>(&Planner::get_arclen_traj));

  py::class_<map_util::OccupancyGrid>(m, "OccupancyGrid", py::module_local())
      .def(py::init<>())
      .def(py::init<int, int, double, double, double,
                    std::vector<unsigned char> &,
                    const std::vector<unsigned char> &,
                    const std::vector<unsigned char> &>())
      .def("is_occupied", py::overload_cast<double, double, const std::string&>(
                              &map_util::OccupancyGrid::is_occupied, py::const_))
      .def("get_origin", &map_util::OccupancyGrid::get_origin)
      .def("world_to_map", &map_util::OccupancyGrid::world_to_map)
      .def("cells_to_index", &map_util::OccupancyGrid::cells_to_index)
      .def("get_cost", py::overload_cast<unsigned int, const std::string &>(
                           &map_util::OccupancyGrid::get_cost))
      .def("update", py::overload_cast<int, int, double, double, double,
                                       const std::vector<unsigned char> &,
                                       const std::vector<unsigned char> &,
                                       const std::vector<unsigned char> &>(
                         &map_util::OccupancyGrid::update));
}
