#include <robust_fast_navigation/planar_ph.h>

#include <array>

#include "gurobi_c++.h"
#include "robust_fast_navigation/rfn_types.h"

namespace planar_ph
{
PHSegment::PHSegment() {}

PHSegment::~PHSegment() {}

GQEVec PHSegment::getPos(double t)
{
    std::vector<GQEVec> p = {getP0(), getP1(), getP2(), getP3(), getP4(), getP5()};

    return evalBezier(p, t);
}

GQEVec PHSegment::getVel(double t)
{
    std::vector<GQEVec> v = {getV0(), getV1(), getV2(), getV3(), getV4()};
    return evalBezier(v, t);
}

GQEVec PHSegment::getAcc(double t)
{
    std::vector<GQEVec> a = {getA0(), getA1(), getA2(), getA3()};
    return evalBezier(a, t);
}

GQEVec PHSegment::getP0() { return GQEVec({_start[0], _start[1]}); }

GQEVec PHSegment::getP1()
{
    // u0u0 - v0v0, 2u0v0
    GRBQuadExpr px = _x[0][0] * _x[0][0] - _x[0][1] * _x[0][1];
    GRBQuadExpr py = 2 * _x[0][0] * _x[0][1];

    GQEVec p1 = getP0() + (1. / 5.) * GQEVec({px, py});

    return p1;
}

GQEVec PHSegment::getP2()
{
    // u0u1 - v0v1, u0v1 + u1v0
    GRBQuadExpr px = _x[0][0] * _x[1][0] - _x[0][1] * _x[1][1];
    GRBQuadExpr py = _x[0][0] * _x[1][1] + _x[1][0] * _x[0][1];

    GQEVec p2 = getP1() + (1. / 5.) * GQEVec({px, py});

    return p2;
}

GQEVec PHSegment::getP3()
{
    // u1*u1 - v1*v1, u1*v2 + u2*v1
    GRBQuadExpr px1 = _x[1][0] * _x[1][0] - _x[1][1] * _x[1][1];
    GRBQuadExpr py1 = _x[1][0] * _x[2][1] + _x[2][0] * _x[1][1];

    // u0u2 - v0v2, u0v2 + u2v0
    GRBQuadExpr px2 = _x[0][0] * _x[2][0] - _x[0][1] * _x[2][1];
    GRBQuadExpr py2 = _x[0][0] * _x[2][1] + _x[2][0] * _x[0][1];

    GQEVec p3 = getP2() + (2. / 15.) * GQEVec({px1, py1}) + (1. / 15.) * GQEVec({px2, py2});

    return p3;
}

GQEVec PHSegment::getP4()
{
    // u1u2 - v1v2, u1v2 + u2v1
    GRBQuadExpr px = _x[1][0] * _x[2][0] - _x[1][1] * _x[2][1];
    GRBQuadExpr py = _x[1][0] * _x[2][1] + _x[2][0] * _x[1][1];

    GQEVec p4 = getP3() + (1. / 5.) * GQEVec({px, py});

    return p4;
}

GQEVec PHSegment::getP5()
{
    // u2*u2 - v2*v2, 2u2*v2
    GRBQuadExpr px = _x[2][0] * _x[2][0] - _x[2][1] * _x[2][1];
    GRBQuadExpr py = 2 * _x[2][0] * _x[2][1];

    GQEVec p5 = getP4() + (1. / 5.) * GQEVec({px, py});

    return p5;
}

GRBQuadExpr PHSegment::getSigma(int i)
{
    int lower = std::max(0, i - 2);
    int upper = std::min(2, i);

    double mcni = _binom4[i];
    GRBQuadExpr ret;
    for (int j = lower; j <= upper; ++j)
    {
        double mcj  = _binom2[j];
        double mcij = _binom2[i - j];
        ret += (mcj * mcij / mcni) * (_x[j][0] * _x[i - j][0] + _x[j][1] * _x[i - j][1]);
    }

    return ret;
}

GRBQuadExpr PHSegment::getS(int i)
{
    GRBQuadExpr ret;
    for (int j = 0; j < i; ++j)
    {
        ret += (1. / 5.) * getSigma(j);
    }
    return ret;
}

GQEVec PHSegment::getV0() { return 5 * (getP1() - getP0()); }
GQEVec PHSegment::getV1() { return 5 * (getP2() - getP1()); }
GQEVec PHSegment::getV2() { return 5 * (getP3() - getP2()); }
GQEVec PHSegment::getV3() { return 5 * (getP4() - getP3()); }
GQEVec PHSegment::getV4() { return 5 * (getP5() - getP4()); }

GQEVec PHSegment::getA0() { return 4 * (getV1() - getV0()); }
GQEVec PHSegment::getA1() { return 4 * (getV2() - getV1()); }
GQEVec PHSegment::getA2() { return 4 * (getV3() - getV2()); }
GQEVec PHSegment::getA3() { return 4 * (getV4() - getV3()); }

GQEVec PHSegment::evalBezier(const std::vector<GQEVec>& coeffs, double t)
{
    int n = coeffs.size() - 1;
    std::vector<double> inv_t_vals(n + 1);
    inv_t_vals[0] = 1;
    inv_t_vals[1] = (1. - t);

    for (int i = 2; i <= n; ++i)
    {
        inv_t_vals[i] = inv_t_vals[i - 1] * inv_t_vals[1];
    }

    const double* binom;
    if (n == 5)
        binom = _binom5.data();
    else if (n == 4)
        binom = _binom4.data();
    else if (n == 3)
        binom = _binom3.data();
    else
        throw std::runtime_error("PHSegment::evalBezier: n must be 3, 4, or 5");

    double t_pow = t;
    GQEVec ret   = coeffs[0] * inv_t_vals[n];
    for (int i = 1; i <= n; ++i)
    {
        ret = ret + coeffs[i] * binom[i] * inv_t_vals[n - i] * t_pow;
        t_pow *= t;
    }

    return ret;
}

PHContour::PHContour(int n_segments)
{
    _segments.reserve(n_segments);
    for (int i = 0; i < n_segments; ++i)
    {
        _segments.emplace_back();
    }
}

PHContour::PHContour(const std::vector<PHSegment>& segments) : _segments(segments) {}

GQEVec PHContour::getPos(double t)
{
    int m = (int)t;
    return _segments[m].getPos(t - m);
}

GQEVec PHContour::getVel(double t)
{
    int m = (int)t;
    return _segments[m].getVel(t - m);
}

GQEVec PHContour::getAcc(double t)
{
    int m = (int)t;
    return _segments[m].getAcc(t - m);
}

PHContourSolver::PHContourSolver()
{
    _n_segments = 0;
    _model      = std::make_unique<GRBModel>(_env);
}

PHContourSolver::~PHContourSolver() {}

void PHContourSolver::setStart(const solver_state& state) { _start = state; }

void PHContourSolver::setGoal(const solver_state& state) { _goal = state; }

void PHContourSolver::setPolytopes(const std::vector<Eigen::MatrixX3d>& polytopes)
{
    _polytopes  = polytopes;
    _n_segments = polytopes.size();

    _contour = PHContour(_n_segments);
}

bool PHContourSolver::optimize()
{
    _model->update();
    try
    {
        _model->optimize();
    }
    catch (GRBException& e)
    {
        std::cerr << "[Contour Solver] Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
        return false;
    }

    int status = _model->get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL)
    {
        /*_model->computeIIS();*/
        /*_model->write("/home/bezzo/catkin_ws/grb_dump.ilp");*/
        /*_model->reset();*/
        /*_model->feasRelax(1, false, false, true);*/
        /**/
        /*_model->optimize();*/
        /**/
        /*for (int i = 0; i < _model->get(GRB_IntAttr_NumConstrs); i++)*/
        /*{*/
        /*    GRBConstr constr = _model->getConstr(i);*/
        /*    double slack     = constr.get(GRB_DoubleAttr_Slack);*/
        /*    if (slack > 1e-6)*/
        /*    {  // Show only significant relaxations*/
        /*        std::cout << "Constraint " << constr.get(GRB_StringAttr_ConstrName)*/
        /*                  << " relaxed by " << slack << std::endl;*/
        /*    }*/
        /*}*/

        std::cerr << "[Contour Solver] Optimization was not successful. Status: " << status
                  << std::endl;
        /*exit(0);*/
        return false;
    }

    /*auto constrs = _model->getConstrs();*/
    /*for (int i = 0; i < _model->get(GRB_IntAttr_NumConstrs); ++i)*/
    /*{*/
    /*    try*/
    /*    {*/
    /*        GRBConstr constr = constrs[i];*/
    /*        std::cout << "Constraint " << i << ": " << constr.get(GRB_StringAttr_ConstrName)*/
    /*                  << " " << constr.get(GRB_CharAttr_Sense) << " "*/
    /*                  << constr.get(GRB_DoubleAttr_RHS) << std::endl;*/
    /*    }*/
    /*    catch (GRBException &e)*/
    /*    {*/
    /*        std::cerr << "[Contour Solver] Error code = " << e.getErrorCode() << std::endl;*/
    /*        std::cerr << e.getMessage() << std::endl;*/
    /*    }*/
    /*}*/
    /**/
    /*delete[] constrs;*/

    return true;
}

void PHContourSolver::initVars()
{
    std::string names = "uv";
    std::string axes  = "012";

    for (int i = 0; i < _n_segments; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            std::vector<GRBVar> coeff_vars;
            for (int k = 0; k < 2; ++k)
            {
                // bounding these terms helps gurobi out a lot, setting bounds high
                std::string var_name = std::string(1, names[k]) + axes[j] + std::to_string(i);
                coeff_vars.push_back(
                    _model->addVar(-1e4, 1e4, 0, GRB_CONTINUOUS, var_name.c_str()));

                std::cout << "var name is " << var_name << std::endl;
            }
            _contour[i]._x[j] = GVec({coeff_vars[0], coeff_vars[1]});
        }
    }
}

void PHContourSolver::setup()
{
    std::cout << "[Contour Solver] Setting up the model" << std::endl;
    initVars();
    std::cout << "[Contour Solver] set vars" << std::endl;

    /*setPolytopeCons();*/
    std::cout << "[Contour Solver] set polys" << std::endl;
    setStartCons();
    std::cout << "[Contour Solver] set start" << std::endl;
    setGoalCons();
    std::cout << "[Contour Solver] set goal" << std::endl;
    setContinuityCons();
    std::cout << "[Contour Solver] set continuity" << std::endl;

    setObjective();

    /*_cb.set_timeout(.15);*/
    _cb.set_timeout(10);
    _model->setCallback(&_cb);
    /*_model->set(GRB_IntParam_NumericFocus, 3);*/
}

void PHContourSolver::setObjective()
{
    GRBQuadExpr obj = 0;
    for (int i = 0; i < _n_segments; ++i)
    {
        obj += _contour[i].getS(5);
    }

    _model->setObjective(obj, GRB_MINIMIZE);
}

void PHContourSolver::setStartCons()
{
    // Remove previous
    /*for (int i = 0; i < _start_constraints.size(); i++)*/
    /*{*/
    /*    _model->remove(_start_constraints[i]);*/
    /*}*/
    /**/
    /*_start_constraints.clear();*/

    _contour[0]._start = Eigen::Vector2d(_start.pos[0], _start.pos[1]);

    /*std::cout << "getting start pos" << _start.pos.transpose() << std::endl;*/
    /*GQEVec pos = _contour[0].getPos(0);*/
    /*std::cout << "getting start vel" << std::endl;*/
    /*GQEVec vel = _contour[0].getVel(0);*/
    /*std::cout << "getting start acc" << std::endl;*/
    /*GQEVec acc = _contour[0].getAcc(0);*/
    /**/
    /*for (int i = 0; i < pos.size(); ++i)*/
    /*{*/
    /*    _start_constraints.push_back(_model->addQConstr(_start.pos[i], GRB_EQUAL, pos[i],*/
    /*                                                    "Start_pos_axis" +
     * std::to_string(i)));*/
    /*}*/
}

void PHContourSolver::setGoalCons()
{
    // Remove previous
    for (int i = 0; i < _goal_constraints.size(); i++)
    {
        _model->remove(_goal_constraints[i]);
    }

    _goal_constraints.clear();

    GQEVec pos = _contour.back().getPos(1);
    GQEVec vel = _contour.back().getVel(1);
    GQEVec acc = _contour.back().getAcc(1);

    for (int i = 0; i < pos.size(); ++i)
    {
        _goal_constraints.push_back(_model->addQConstr(_goal.pos[i], GRB_EQUAL, pos[i],
                                                       "End_pos_axis" + std::to_string(i)));
    }
}

void PHContourSolver::setPolytopeCons()
{
    // Remove previous
    for (int i = 0; i < _polytope_constraints.size(); i++)
    {
        _model->remove(_polytope_constraints[i]);
    }
    _polytope_constraints.clear();

    // Add new
    for (int i = 0; i < _polytopes.size(); ++i)
    {
        std::cout << "poly i " << i << std::endl;
        int cols          = _polytopes[i].cols();
        Eigen::MatrixXd A = _polytopes[i].leftCols(cols - 1);
        Eigen::VectorXd b = -_polytopes[i].rightCols(1);

        std::cout << "set A and b" << std::endl;

        PHSegment segment = _contour[i];
        GQEVec p0         = segment.getP0();
        std::cout << "segment 0" << std::endl;
        GQEVec p1 = segment.getP1();
        std::cout << "segment 1" << std::endl;
        GQEVec p2 = segment.getP2();
        std::cout << "segment 2" << std::endl;
        GQEVec p3 = segment.getP3();
        std::cout << "segment 3" << std::endl;
        GQEVec p4 = segment.getP4();
        std::cout << "segment 4" << std::endl;
        GQEVec p5 = segment.getP5();
        std::cout << "segment 5" << std::endl;

        std::cout << "got CPs" << std::endl;

        std::cout << A << std::endl;
        std::cout << b.transpose() << std::endl;

        GQEVec Acp0 = A * p0;
        std::cout << "finished mult 0" << std::endl;
        GQEVec Acp1 = A * p1;
        GQEVec Acp2 = A * p2;
        GQEVec Acp3 = A * p3;
        GQEVec Acp4 = A * p4;
        GQEVec Acp5 = A * p5;
        std::cout << "finished all mults" << std::endl;

        for (int j = 0; j < A.rows(); ++j)
        {
            /*GRBQConstr c0 = _model->addQConstr(*/
            /*    Acp0[j], GRB_LESS_EQUAL, b(j),*/
            /*    "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p0");*/
            /**/
            /*_polytope_constraints.push_back(c0);*/

            _polytope_constraints.push_back(_model->addQConstr(
                Acp1[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p1"));
            _polytope_constraints.push_back(_model->addQConstr(
                Acp2[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p2"));
            _polytope_constraints.push_back(_model->addQConstr(
                Acp3[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p3"));
            _polytope_constraints.push_back(_model->addQConstr(
                Acp4[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p4"));
            _polytope_constraints.push_back(_model->addQConstr(
                Acp5[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p5"));
        }
    }
}

void PHContourSolver::setContinuityCons()
{
    // Remove previous
    for (int i = 0; i < _cont_constraints.size(); i++)
    {
        _model->remove(_cont_constraints[i]);
    }

    _cont_constraints.clear();

    for (int i = 0; i < _n_segments - 1; ++i)
    {
        GQEVec pos0 = _contour[i].getPos(1);
        GQEVec vel0 = _contour[i].getVel(1);
        GQEVec acc0 = _contour[i].getAcc(1);

        GQEVec pos1 = _contour[i + 1].getPos(0);
        GQEVec vel1 = _contour[i + 1].getVel(0);
        GQEVec acc1 = _contour[i + 1].getAcc(0);

        for (int j = 0; j < pos0.size(); ++j)
        {
            _cont_constraints.push_back(_model->addQConstr(
                pos0.x[j], GRB_EQUAL, pos1.x[j], "Cont_pos_axis" + std::to_string(j)));
            _cont_constraints.push_back(_model->addQConstr(
                vel0.x[j], GRB_EQUAL, vel1.x[j], "Cont_vel_axis" + std::to_string(j)));
            _cont_constraints.push_back(_model->addQConstr(
                acc0.x[j], GRB_EQUAL, acc1.x[j], "Cont_acc_axis" + std::to_string(j)));
        }
    }
}

}  // namespace planar_ph
