#include <robust_fast_navigation/JPS.h>
#include <robust_fast_navigation/planner_core.h>

#include <memory>

#include "costmap_2d/cost_values.h"
#include "robust_fast_navigation/faster_wrapper.h"
#include "robust_fast_navigation/gcopter_wrapper.h"
#include "robust_fast_navigation/solver_base.h"

Planner::Planner()
{
    _is_map_set   = false;
    _is_goal_set  = false;
    _simplify_jps = false;
    _is_start_set = false;
    _plan_in_free = false;

    _solver = std::make_unique<FasterWrapper>();
}

Planner::~Planner() {}

void Planner::set_params(const planner_params_t &params)
{
    _params = params;

    if (_params.SOLVER == "faster")
        _solver = std::make_unique<FasterWrapper>();
    else if (_params.SOLVER == "gcopter")
        _solver = std::make_unique<GcopterWrapper>();
    else
    {
        std::cout << termcolor::red << "[Planner Core] Solver param value '" << _params.SOLVER
                  << "' not recognized! Set to either 'faster' or 'gcopter'" << termcolor::reset
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

    double limits[3]        = {v_max, a_max, j_max};
    double factor_init      = params.DT_FACTOR_INIT;
    double factor_final     = params.DT_FACTOR_FINAL;
    double factor_increment = params.DT_FACTOR_INCREMENT;

    _solver->set_params(params);
}

void Planner::set_start(const Eigen::MatrixXd &start)
{
    _start        = start;
    _is_start_set = true;
}

void Planner::set_goal(const Eigen::MatrixXd &goal)
{
    _goal        = goal;
    _is_goal_set = true;
}

void Planner::set_costmap(const map_util::occupancy_grid_t &map)
{
    _map        = map;
    _is_map_set = true;
}

bool Planner::plan(double horizon, std::vector<Eigen::Vector2d> &jpsPath,
                   std::vector<Eigen::MatrixX4d> &hPolys)
{
    if (!_is_start_set || !_is_goal_set || !_is_map_set)
    {
        std::cout << termcolor::red << "[Planner Core] missing start, goal or costmap"
                  << termcolor::reset << std::endl;
        return false;
    }

    /*************************************
    ************ PERFORM  JPS ************
    **************************************/

    jps::JPSPlan jps;
    unsigned int sX, sY, eX, eY;

    // potentially change this to odom instead of start
    std::vector<unsigned int> start_cells = _map.world_to_map(_start(0, 0), _start(1, 0));
    sX                                    = start_cells[0];
    sY                                    = start_cells[1];
    // std::cout << "startind is " << _map.cells_to_index(sX, sY) << std::endl;
    // std::cout << "sX = " << sX << std::endl;
    // std::cout << "sY = " << sY << std::endl;

    std::vector<unsigned int> goal_cells = _map.world_to_map(_goal(0), _goal(1));
    eX                                   = goal_cells[0];
    eY                                   = goal_cells[1];

    // std::cout << "eX = " << eX << std::endl;
    // std::cout << "eY = " << eY << std::endl;

    jps.set_start(sX, sY);
    jps.set_destination(eX, eY);
    jps.set_occ_value(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

    jps.set_map(_map.get_data(), _map.width, _map.height, _map.origin_x, _map.origin_y,
                _map.resolution);

    int jps_status = jps.JPS();

    // try one more time without inflated obstacles...
    if (jps_status == IN_OCCUPIED_SPACE)
    {
        jps.set_occ_value(costmap_2d::LETHAL_OBSTACLE);
        jps_status = jps.JPS();
    }

    std::vector<Eigen::Vector2d> oldJps = jpsPath;

    jpsPath = jps.getPath(_simplify_jps);

    if (jpsPath.size() < 2)
    {
        std::cout << termcolor::red << "[Planner Core] JPS failed" << termcolor::reset
                  << std::endl;
        return false;
    }

    // check if old jps path has value
    if (oldJps.size() > 0 && !JPSIntersectObs(oldJps))
    {
        // if new path angle is much different from old path with nearly same
        // length, use old path
        Eigen::Vector2d oldDir = oldJps[oldJps.size() - 1] - oldJps[0];
        Eigen::Vector2d newDir = jpsPath[jpsPath.size() - 1] - jpsPath[0];

        oldDir = oldDir / oldDir.norm();
        newDir = newDir / newDir.norm();

        double angle = acos(oldDir.dot(newDir)) * 180 / M_PI;
        std::cout << termcolor::on_magenta << "Old jps given, and doesn't intersect obs"
                  << std::endl;
        std::cout << "angle between old and new path is " << angle << termcolor::reset
                  << std::endl;

        if (angle > 30)
        {
            // if new path is longer or barely shorter than old path, use old
            // path
            double oldLen = 0;
            for (int i = 0; i < oldJps.size() - 1; ++i)
            {
                oldLen += (oldJps[i + 1] - oldJps[i]).norm();
            }

            double newLen = 0;
            for (int i = 0; i < jpsPath.size() - 1; ++i)
            {
                newLen += (jpsPath[i + 1] - jpsPath[i]).norm();
            }

            if (newLen > .98 * oldLen)
            {
                jpsPath = oldJps;
            }
            std::cout << termcolor::on_magenta << "new len is " << newLen << "\nand old len is "
                      << oldLen << termcolor::reset << std::endl;
        }
    }

    /*************************************
    ************* REFINE JPS *************
    **************************************/

    std::vector<Eigen::Vector2d> newJPSPath;

    if (_plan_in_free)
    {
        jpsPath = getJPSInFree(jpsPath);
    }

    double jps_path_length = 0;
    for (int i = 0; i < jpsPath.size() - 1; ++i)
    {
        jps_path_length += (jpsPath[i + 1] - jpsPath[i]).norm();
    }

    std::cout << "JPS path lenth is " << jps_path_length << " / " << horizon << std::endl;
    if (jps.truncateJPS(jpsPath, newJPSPath, horizon))
    {
        jpsPath = newJPSPath;
    }

    _goal << Eigen::Vector3d(jpsPath.back()[0], jpsPath.back()[1], 0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    if (jpsPath.size() == 0)
    {
        std::cout << termcolor::red << "[Planner Core] JPS modifications failed"
                  << termcolor::reset << std::endl;
        return false;
    }

    /*************************************
    ********* GENERATE POLYTOPES *********
    **************************************/

    hPolys.clear();
    if (!corridor::createCorridorJPS(jpsPath, _map, hPolys, _start, _goal))
    {
        std::cout << termcolor::red << "[Planner Core] Corridor creation failed"
                  << termcolor::reset << std::endl;
        return false;
    }

    bool is_in_corridor = false;

    // if adjacent polytopes don't overlap, don't plan
    for (int p = 0; p < hPolys.size() - 1; p++)
    {
        if (!geo_utils::overlap(hPolys[p], hPolys[p + 1]))
        {
            // ROS_ERROR("CORRIDOR IS NOT FULLY CONNECTED");
            return false;
        }

        if (!is_in_corridor)
            is_in_corridor =
                corridor::isInPoly(hPolys[p], Eigen::Vector2d(_start(0, 0), _start(1, 0)));
    }

    // if(corridor::union_corridor(hPolys, _cgal_border, _corridor_boundary))
    //     std::cout << "we did it!!!" << std::endl;

    /*************************************
    ******** GENERATE  TRAJECTORY ********
    **************************************/

    if (!_solver->setup(_start, _goal, hPolys))
    {
        std::cout << termcolor::red << "[Planner Core] Solver setup failed" << termcolor::reset
                  << std::endl;
        return false;
    }

    // time trajectory generation
    ros::Time start_solve = ros::Time::now();
    if (!_solver->solve())
    {
        std::cout << termcolor::red << "[Planner Core] Generating trajectory failed"
                  << termcolor::reset << std::endl;
        return false;
    }
    else
    {
        std::cout << termcolor::green << "[Planner Core] Solver found trajectory, time: "
                  << (ros::Time::now() - start_solve).toSec() << termcolor::reset << std::endl;
    }

    std::vector<rfn_state_t> traj = _solver->get_trajectory();

    // ensure trajectory does not overlap lethal obstacles
    for (int i = 0; i < traj.size(); ++i)
    {
        Eigen::Vector2d pos = traj[i].pos.head(2);
        if (_map.is_occupied(pos[0], pos[1]))
        {
            std::cout << termcolor::red << "[Planner Core] Trajectory overlaps obstacle"
                      << termcolor::reset << std::endl;
            return false;
        }
    }

    _is_start_set = false;
    _is_goal_set  = false;
    _is_map_set   = false;

    _old_goal = _goal;

    return true;
}

std::vector<rfn_state_t> Planner::get_trajectory() { return _solver->get_trajectory(); }

std::vector<rfn_state_t> Planner::get_arclen_traj()
{
    std::vector<double> ss;
    std::vector<double> xs;
    std::vector<double> ys;

    bool status = reparam_traj(ss, xs, ys);

    if (!status) return {};

    std::vector<rfn_state_t> ret;
    ret.reserve(ss.size());

    for (int i = 0; i < ss.size(); ++i)
    {
        rfn_state_t &x = ret.emplace_back();
        x.pos(0)       = xs[i];
        x.pos(1)       = ys[i];
        x.t            = ss[i];
    }

    return ret;
}

std::vector<Eigen::Vector3d> Planner::get_cps()
{
    std::vector<Eigen::Vector3d> ret;
    // for (int interval = 0; interval < _solver.N_; interval++)
    // {
    //     std::vector<GRBLinExpr> cp0 = _solver.getCP0(interval);
    //     std::vector<GRBLinExpr> cp1 = _solver.getCP1(interval);
    //     std::vector<GRBLinExpr> cp2 = _solver.getCP2(interval);
    //     std::vector<GRBLinExpr> cp3 = _solver.getCP3(interval);

    //     Eigen::Vector3d cp0_vec(cp0[0].getValue(), cp0[1].getValue(), cp0[2].getValue());
    //     Eigen::Vector3d cp1_vec(cp1[0].getValue(), cp1[1].getValue(), cp1[2].getValue());
    //     Eigen::Vector3d cp2_vec(cp2[0].getValue(), cp2[1].getValue(), cp2[2].getValue());
    //     Eigen::Vector3d cp3_vec(cp3[0].getValue(), cp3[1].getValue(), cp3[2].getValue());

    //     ret.push_back(cp0_vec);
    //     ret.push_back(cp1_vec);
    //     ret.push_back(cp2_vec);
    //     ret.push_back(cp3_vec);
    // }

    return ret;
}

bool Planner::JPSIntersectObs(const std::vector<Eigen::Vector2d> &path)
{
    if (!_is_map_set)
    {
        std::cout << termcolor::yellow << "[Planner] costmap not yet set" << termcolor::reset
                  << std::endl;
        return false;
    }

    for (int i = 0; i < path.size() - 1; i++)
    {
        double x, y;
        unsigned int sx, sy, ex, ey;

        std::vector<unsigned int> start_cells = _map.world_to_map(path[i][0], path[i][1]);
        sx                                    = start_cells[0];
        sy                                    = start_cells[1];

        std::vector<unsigned int> end_cells = _map.world_to_map(path[i + 1][0], path[i + 1][1]);
        ex                                  = end_cells[0];
        ey                                  = end_cells[1];

        _map.raycast(sx, sy, ex, ey, x, y,
                     {costmap_2d::LETHAL_OBSTACLE, costmap_2d::INSCRIBED_INFLATED_OBSTACLE});

        // if x,y does not reach the end of the ray, we hit an unknown cell
        Eigen::Vector2d p(x, y);

        if ((path[i + 1] - p).norm() > _map.resolution)
        {
            std::vector<unsigned int> p_cells = _map.world_to_map(p[0], p[1]);
            return true;
        }
    }

    return false;
}

std::vector<Eigen::Vector2d> Planner::getJPSInFree(const std::vector<Eigen::Vector2d> &path)
{
    if (!_is_map_set)
    {
        std::cout << termcolor::yellow << "[Planner] costmap not yet set" << termcolor::reset
                  << std::endl;
        return path;
    }

    std::vector<Eigen::Vector2d> ret;

    if (path.size() < 2)
    {
        std::cout << termcolor::yellow << "[Planner] JPS only contains one point"
                  << termcolor::reset << std::endl;
        return path;
    }

    // walk along jps and find first unknown cell

    double x, y;
    unsigned int sx, sy, ex, ey;

    ret.push_back(path[0]);

    // raycast along JPS until we hit an unknown cell
    for (int i = 0; i < path.size() - 1; i++)
    {
        std::vector<unsigned int> start_cells = _map.world_to_map(path[i][0], path[i][1]);
        sx                                    = start_cells[0];
        sy                                    = start_cells[1];

        std::vector<unsigned int> end_cells = _map.world_to_map(path[i + 1][0], path[i + 1][1]);
        ex                                  = end_cells[0];
        ey                                  = end_cells[1];

        _map.raycast(sx, sy, ex, ey, x, y, {costmap_2d::NO_INFORMATION});

        // if x,y does not reach the end of the ray, we hit an unknown cell
        Eigen::Vector2d p(x, y);

        if ((path[i + 1] - p).norm() > 1e-3)
        {
            ret.push_back(p);
            break;
        }

        ret.push_back(path[i + 1]);
    }

    return ret;
}

bool Planner::reparam_traj(std::vector<double> &ss, std::vector<double> &xs,
                           std::vector<double> &ys)
{
    std::vector<rfn_state_t> traj = _solver->get_trajectory();
    if (traj.size() == 0) return false;

    double traj_duration = traj.back().t;

    double total_length = compute_arclen(0, traj_duration);

    double M  = 20;
    double ds = total_length / M;

    ss.resize(M + 1);
    xs.resize(M + 1);
    ys.resize(M + 1);

    double previous_ti = 0;
    for (int i = 0; i <= M; ++i)
    {
        double s = i * ds;

        double ti = binary_search(s, previous_ti, traj_duration, 1e-3);

        // std::cout << "s is " << s << " and ti is " << ti << std::endl;

        ss[i] = s;
        xs[i] = _solver->get_pos(ti, 0);
        ys[i] = _solver->get_pos(ti, 1);

        previous_ti = ti;
    }

    return true;
}

double Planner::binary_search(double dl, double start, double end, double tolerance)
{
    double t_left  = start;
    double t_right = end;

    double prev_s = 0;
    double s      = -1000;

    while (fabs(prev_s - s) > tolerance)
    {
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

double Planner::compute_arclen(double t0, double tf)
{
    double s  = 0.0;
    double dt = (tf - t0) / 100.;

    double prev_dx = _solver->get_vel(t0, 0);
    double prev_dy = _solver->get_vel(t0, 1);

    for (double t = t0; t < tf; t += dt)
    {
        double dx, dy;
        dx = _solver->get_vel(t, 0);
        dy = _solver->get_vel(t, 1);

        // s += std::sqrt(dx * dx + dy * dy) * dt;
        s += .5 *
             (std::sqrt(dx * dx + dy * dy) + std::sqrt(prev_dx * prev_dx + prev_dy * prev_dy)) *
             dt;

        prev_dx = dx;
        prev_dy = dy;
    }

    return s;
}
