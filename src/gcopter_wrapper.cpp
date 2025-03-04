#include <robust_fast_navigation/gcopter_wrapper.h>

GcopterWrapper::GcopterWrapper() {}

GcopterWrapper::~GcopterWrapper() {}

bool GcopterWrapper::setup(const Eigen::MatrixXd& start, const Eigen::MatrixXd& end,
                           const std::vector<Eigen::MatrixX4d>& polys)
{
    // gcopter does not use jerk
    Eigen::Matrix3d start_pva = start.block(0, 0, 3, 3);
    Eigen::Matrix3d end_pva   = end.block(0, 0, 3, 3);

    std::cout << "start pva: \n" << start_pva << std::endl;
    std::cout << "end pva: \n" << end_pva << std::endl;

    Eigen::VectorXd magnitudeBounds(5);
    Eigen::VectorXd penaltyWeights(5);
    Eigen::VectorXd physicalParams(6);
    magnitudeBounds(0) = 4.0;
    magnitudeBounds(1) = 2.1;
    magnitudeBounds(2) = 1.05;
    magnitudeBounds(3) = 2.0;
    magnitudeBounds(4) = 12.0;
    penaltyWeights(0)  = 1e4;
    penaltyWeights(1)  = 1e4;
    penaltyWeights(2)  = 1e4;
    penaltyWeights(3)  = 1e4;
    penaltyWeights(4)  = 1e5;
    physicalParams(0)  = .61;
    physicalParams(1)  = 9.8;
    physicalParams(2)  = 0;
    physicalParams(3)  = 0;
    physicalParams(4)  = 0;
    physicalParams(5)  = .0001;

    return _solver.setup(20.0, start_pva, end_pva, polys, 1e6, 1e-2, 16, magnitudeBounds,
                         penaltyWeights, physicalParams);
}

bool GcopterWrapper::solve()
{
    _traj.clear();
    bool success = !std::isinf(_solver.optimize(_traj, 1e-5));

    if (success) _params.N_SEGMENTS = _traj.getPieceNum();

    return success;
}

std::vector<rfn_state_t> GcopterWrapper::get_trajectory()
{
    // convert trajectory<D> into vector of rfn_state_t
    double duration = _traj.getTotalDuration();

    std::vector<rfn_state_t> ret;
    ret.reserve(duration / _params.SOLVER_TRAJ_DT);

    for (double t = 0; t < duration; t += _params.SOLVER_TRAJ_DT)
    {
        Eigen::Vector3d pos = _traj.getPos(t);
        Eigen::Vector3d vel = _traj.getVel(t);
        Eigen::Vector3d acc = _traj.getAcc(t);

        rfn_state_t& rfn_st = ret.emplace_back();
        rfn_st.pos          = pos;
        rfn_st.vel          = vel;
        rfn_st.accel        = acc;
        rfn_st.t            = t;
    }

    return ret;
}

void GcopterWrapper::set_params(const planner_params_t& params) { _params = params; }

double GcopterWrapper::get_pos(double t, int dim)
{
    Eigen::Vector3d pos = _traj.getPos(t);
    return pos(dim);
}

double GcopterWrapper::get_vel(double t, int dim)
{
    Eigen::Vector3d vel = _traj.getVel(t);
    return vel(dim);
}
