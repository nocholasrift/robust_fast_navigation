
#include <robust_fast_navigation/faster_wrapper.h>

FasterWrapper::FasterWrapper() {}

FasterWrapper::~FasterWrapper() {}

void FasterWrapper::set_params(const planner_params_t& params)
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

    _solver.setN(params.N_SEGMENTS);
    _solver.createVars();
    _solver.setDC(params.SOLVER_TRAJ_DT);
    _solver.setBounds(limits);
    _solver.setWMax(params.W_MAX);
    _solver.setForceFinalConstraint(params.FORCE_FINAL_CONSTRAINT);
    _solver.setFactorInitialAndFinalAndIncrement(factor_init, factor_final, factor_increment);
    _solver.setThreads(params.N_THREADS);
    _solver.setVerbose(params.VERBOSE);
    _solver.setUseMinvo(params.USE_MINVO);
    _solver.setMaxSolverTime(params.MAX_SOLVE_TIME);

    _params = params;
}

bool FasterWrapper::setup(const Eigen::MatrixXd& start, const Eigen::MatrixXd& end,
                          const std::vector<Eigen::MatrixX4d>& polys)
{
    // set initial and final positions for solver
    faster::state initialState;
    faster::state finalState;

    initialState.setPos(start(0, 0), start(1, 0), start(2, 0));
    initialState.setVel(start(0, 1), start(1, 1), start(2, 1));
    initialState.setAccel(start(0, 2), start(1, 2), start(2, 2));
    initialState.setJerk(start(0, 3), start(1, 3), start(2, 3));

    finalState.setPos(end.col(0));
    finalState.setVel(end.col(1));
    finalState.setAccel(end.col(2));
    finalState.setJerk(end.col(3));

    _solver.setX0(initialState);
    _solver.setXf(finalState);

    // set polygons
    _solver.setPolytopes(polys);

    return true;
}

bool FasterWrapper::solve()
{
    bool success = _solver.genNewTraj();

    if (success) _solver.fillX();

    // populate DT param field since it changes per solve
    _params.DT = _solver.dt_;

    return success;
}

std::vector<rfn_state_t> FasterWrapper::get_trajectory()
{
    std::vector<rfn_state_t> ret;
    ret.reserve(_solver.X_temp_.size());
    for (const faster::state& st : _solver.X_temp_)
    {
        rfn_state_t& rfn_st = ret.emplace_back();
        rfn_st.pos          = st.pos;
        rfn_st.vel          = st.vel;
        rfn_st.accel        = st.accel;
        rfn_st.jerk         = st.jerk;
        rfn_st.t            = st.t;
    }

    return ret;
}

double FasterWrapper::get_pos(double t, int dim)
{
    double ret;

    // find segment
    double dt   = _solver.dt_;
    int segment = std::min(t / dt, _solver.N_ - 1.);

    try
    {
        ret = _solver.getPos(segment, t - segment * dt, dim).getValue();
    }
    catch (const GRBException& e)
    {
        std::cerr << "[Gurobi Solver] in get_pos" << e.getMessage() << '\n';
        std::cerr << "segment " << segment << std::endl;
        exit(1);
    }
    return ret;
}

double FasterWrapper::get_vel(double t, int dim)
{
    double ret;

    // find segment
    double dt   = _solver.dt_;
    int segment = std::min(t / dt, _solver.N_ - 1.);

    try
    {
        ret = _solver.getVel(segment, t - segment * dt, dim).getValue();
    }
    catch (const GRBException& e)
    {
        std::cerr << "[Gurobi Solver] in get_pos" << e.getMessage() << '\n';
        std::cerr << "segment " << segment << std::endl;
        exit(1);
    }
    return ret;
}
