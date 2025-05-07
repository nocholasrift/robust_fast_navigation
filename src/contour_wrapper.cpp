#include <robust_fast_navigation/contour_wrapper.h>

#include <Eigen/Core>
#include <cppad/core/testvector.hpp>

#define ad_to_double(x) CppAD::Value(CppAD::Var2Par(x))

ContourWrapper::ContourWrapper() {}

ContourWrapper::~ContourWrapper() {}

void ContourWrapper::set_params(const planner_params_t& params)
{
    // solver params
    double w_max = params.W_MAX;
    double v_max = params.V_MAX;
    double a_max = params.A_MAX;
    double j_max = params.J_MAX;

    double limits[3]        = {v_max, a_max, j_max};
    double factor_init      = params.DT_FACTOR_INIT;
    double factor_final     = params.DT_FACTOR_FINAL;
    double factor_increment = params.DT_FACTOR_INCREMENT;

    std::cout << "[Gurobi Solver] Loaded params: " << std::endl;
    std::cout << "\tW_MAX: " << w_max << std::endl;
    std::cout << "\tV_MAX: " << v_max << std::endl;
    std::cout << "\tA_MAX: " << a_max << std::endl;
    std::cout << "\tJ_MAX: " << j_max << std::endl;
    std::cout << "\tDT_FACTOR_INIT: " << factor_init << std::endl;
    std::cout << "\tDT_FACTOR_FINAL: " << factor_final << std::endl;
    std::cout << "\tDT_FACTOR_INCREMENT: " << factor_increment << std::endl;
    std::cout << "\tMAX_SOLVE_TIME: " << params.MAX_SOLVE_TIME << std::endl;

    // _solver.setN(params.N_SEGMENTS);
    // _solver.createVars();
    // _solver.setDC(params.SOLVER_TRAJ_DT);
    // _solver.setBounds(limits);
    // _solver.setWMax(params.W_MAX);
    // _solver.setForceFinalConstraint(params.FORCE_FINAL_CONSTRAINT);
    // _solver.setFactorInitialAndFinalAndIncrement(factor_init, factor_final,
    // factor_increment); _solver.setThreads(params.N_THREADS);
    // _solver.setVerbose(params.VERBOSE);
    // _solver.setUseMinvo(params.USE_MINVO);
    // _solver.setMaxSolverTime(params.MAX_SOLVE_TIME);

    _params = params;
}

void ContourWrapper::set_map(const map_util::occupancy_grid_t& map) { _solver.setMap(map); }

bool ContourWrapper::setup(const Eigen::MatrixXd& start, const Eigen::MatrixXd& end,
                           const std::vector<Eigen::MatrixX4d>& polys)
{
    // set initial and final positions for solver
    solver_state initialState;
    solver_state finalState;

    initialState.pos = start.col(0);
    initialState.vel = start.col(1);
    /*initialState.accel = start.col(2);*/
    /*initialState.jerk  = start.col(3);*/

    finalState.pos = end.col(0);
    finalState.vel = end.col(1);
    /*finalState.accel = end.col(2);*/
    /*finalState.jerk  = end.col(3);*/

    _solver.setStart(start.col(0).head(2));

    if (start.col(1).head(2).norm() > 1e-3) _solver.setVel(start.col(1).head(2));

    _solver.setGoal(end.col(0).head(2));
    _solver.setSegments(polys.size());

    std::cout << "POLY SIZE IS: " << polys.size() << std::endl;

    // make polys 3d
    std::vector<Eigen::MatrixX3d> newpolys;
    for (const auto& poly : polys)
    {
        Eigen::MatrixX3d newpoly(poly.rows(), 3);
        newpoly.leftCols(2) = poly.leftCols(2);
        newpoly.col(2)      = -poly.col(3);
        newpolys.push_back(newpoly);
    }

    // set polygons
    _solver.setPolys(newpolys);

    return true;
}

bool ContourWrapper::solve()
{
    std::cout << "solvin traj" << std::endl;
    std::vector<double> x0 = _solver.warm_start();
    bool success           = _solver.solve(x0);

    return success;
}

std::vector<rfn_state_t> ContourWrapper::get_trajectory()
{
    std::cout << "getting trajectory" << std::endl;
    double dt = _params.SOLVER_TRAJ_DT;

    PlanarTrajectory<CppAD::AD<double>, CPPAD_TESTVECTOR(CppAD::AD<double>)> contour =
        _solver.getTraj();

    double duration = ad_to_double(contour.getArcLen());

    std::cout << "DURATION IS: " << duration << std::endl;
    std::cout << "DT IS: " << dt << std::endl;

    double t = 0.;
    int n    = duration / dt;
    std::vector<rfn_state_t> ret;
    ret.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        t += dt;
        auto pos = contour.getPos(t);
        auto vel = contour.getVel(t);
        /*auto acc = contour.getAcc(t);*/

        rfn_state_t& rfn_st = ret.emplace_back();
        rfn_st.pos          = Eigen::Vector3d(ad_to_double(pos[0]), ad_to_double(pos[1]), 0);
        rfn_st.vel          = Eigen::Vector3d(ad_to_double(vel[0]), ad_to_double(vel[1]), 0);
        rfn_st.accel        = Eigen::Vector3d::Zero();
        rfn_st.jerk         = Eigen::Vector3d::Zero();
        rfn_st.t            = t;

        /*std::cout << t << "\t" << rfn_st.pos.transpose() << std::endl;*/

        /*std::cout << rfn_st.pos.transpose() << std::endl;*/
        /*for (int i = 0; i < 4; ++i)*/
        /*{*/
        /*    std::cout << contour[0]._x[i][0].get(GRB_DoubleAttr_X) << " ";*/
        /*}*/
        /*std::cout << "\n";*/
    }
    /*exit(0);*/

    return ret;
}

double ContourWrapper::get_pos(double t, int dim)
{
    return ad_to_double(_solver.getTraj().getPos(t)[dim]);
}

double ContourWrapper::get_vel(double t, int dim)
{
    return ad_to_double(_solver.getTraj().getVel(t)[dim]);
}

double ContourWrapper::get_arclen() { return ad_to_double(_solver.getTraj().getArcLen()); }
