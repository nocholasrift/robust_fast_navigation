#include <robust_fast_navigation/contour_wrapper.h>

#include <Eigen/Core>

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

bool ContourWrapper::setup(const Eigen::MatrixXd& start, const Eigen::MatrixXd& end,
                           const std::vector<Eigen::MatrixX4d>& polys)
{
    // set initial and final positions for solver
    solver_state initialState;
    solver_state finalState;

    initialState.pos   = start.col(0);
    initialState.vel   = start.col(1);
    initialState.accel = start.col(2);
    initialState.jerk  = start.col(3);

    finalState.pos   = end.col(0);
    finalState.vel   = end.col(1);
    finalState.accel = end.col(2);
    finalState.jerk  = end.col(3);

    _solver.setStart(initialState);
    _solver.setGoal(finalState);

    // set polygons
    _solver.setPolytopes(polys);

    _params.N_SEGMENTS = polys.size();

    _solver.setup();

    return true;
}

bool ContourWrapper::solve()
{
    std::cout << "solvin traj" << std::endl;
    bool success = _solver.optimize();

    return success;
}

std::vector<rfn_state_t> ContourWrapper::get_trajectory()
{
    std::cout << "getting trajectory" << std::endl;
    double dt       = _params.SOLVER_TRAJ_DT;
    double duration = _params.N_SEGMENTS;

    contour_solver::Contour contour = _solver.getContour();

    std::vector<rfn_state_t> ret;
    ret.reserve(duration / dt);
    for (double t = 0.; t < duration; t += dt)
    {
        contour_solver::GLEVec pos  = contour.getPos(t);
        contour_solver::GLEVec vel  = contour.getVel(t);
        contour_solver::GLEVec acc  = contour.getAcc(t);
        contour_solver::GLEVec jerk = contour.getJerk(t);

        rfn_state_t& rfn_st = ret.emplace_back();
        rfn_st.pos   = Eigen::Vector3d(pos[0].getValue(), pos[1].getValue(), pos[2].getValue());
        rfn_st.vel   = Eigen::Vector3d(vel[0].getValue(), vel[1].getValue(), vel[2].getValue());
        rfn_st.accel = Eigen::Vector3d(acc[0].getValue(), acc[1].getValue(), acc[2].getValue());
        rfn_st.jerk =
            Eigen::Vector3d(jerk[0].getValue(), jerk[1].getValue(), jerk[2].getValue());
        rfn_st.t = t;

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
    return _solver.getContour().getPos(t)[dim].getValue();
}

double ContourWrapper::get_vel(double t, int dim)
{
    return _solver.getContour().getPos(t)[dim].getValue();
}
