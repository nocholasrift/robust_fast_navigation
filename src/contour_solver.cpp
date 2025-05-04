#include <robust_fast_navigation/contour_solver.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "gurobi_c++.h"

namespace contour_solver
{

// segment definition
Segment::Segment()
{
    // for (int i = 0; i < 4; ++i)
    // {
    //     _x[i] = GLEVec({0, 0, 0});
    // }
}

Segment::~Segment() {}

GLEVec Segment::getPos(double t)
{
    return _x[0] * t * t * t + _x[1] * t * t + _x[2] * t + _x[3];
}

GLEVec Segment::getVel(double t) { return 3 * _x[0] * t * t + 2 * _x[1] * t + _x[2]; }

GLEVec Segment::getAcc(double t) { return 6 * _x[0] * t + 2 * _x[1]; }

GLEVec Segment::getJerk(double t) { return 6 * _x[0]; }

GLEVec Segment::getP0() { return 1 * _x[3]; }

GLEVec Segment::getP1() { return (_x[2] + 3 * _x[3]) / 3; }

GLEVec Segment::getP2() { return (_x[1] + 2 * _x[2] + 3 * _x[3]) / 3; }

GLEVec Segment::getP3() { return _x[0] + _x[1] + _x[2] + _x[3]; }

Contour::Contour(const std::vector<Segment> &segments) : _segments(segments) {}

Contour::Contour(int n_segments)
{
    _segments.reserve(n_segments);
    for (int i = 0; i < n_segments; ++i)
    {
        _segments.emplace_back();
    }
}

Segment Contour::getSegment(int m)
{
    if (m < _segments.size())
    {
        return _segments[m];
    }
    else
    {
        std::cerr << "segment out of range" << std::endl;
        return Segment();
    }
}

GLEVec Contour::getPos(double t)
{
    int m = findSegment(t);
    if (m < _segments.size() && m >= 0)
    {
        return _segments[m].getPos(t - m);
    }
    else
        throw std::runtime_error("[Contour] segment out of range" + std::to_string(m) + " " +
                                 std::to_string(_segments.size()));
}

GLEVec Contour::getVel(double t)
{
    int m = findSegment(t);
    if (m < _segments.size() && m >= 0)
    {
        return _segments[m].getVel(t - m);
    }
    else
        throw std::runtime_error("[Contour] segment out of range");
}

GLEVec Contour::getAcc(double t)
{
    int m = findSegment(t);
    if (m < _segments.size() && m >= 0)
    {
        return _segments[m].getAcc(t - m);
    }
    else
        throw std::runtime_error("[Contour] segment out of range");
}

GLEVec Contour::getJerk(double t)
{
    int m = findSegment(t);
    if (m < _segments.size() && m >= 0)
    {
        return _segments[m].getJerk(t - m);
    }
    else
        throw std::runtime_error("[Contour] segment out of range");
}

int Contour::findSegment(double t)
{
    int segment = t;
    if (segment > _segments.size())
    {
        std::cerr << "segment out of range" << std::endl;
        return -1;
    }
    return t;
}

ContourSolver::ContourSolver()
{
    _n_segments = 0;
    _model      = std::make_unique<GRBModel>(_env);
}

ContourSolver::~ContourSolver() {}

void ContourSolver::setStart(const solver_state &state) { _start = state; }

void ContourSolver::setGoal(const solver_state &state) { _goal = state; }

void ContourSolver::setPolytopes(const std::vector<Eigen::MatrixX4d> &polytopes)
{
    _polytopes  = polytopes;
    _n_segments = polytopes.size();

    _contour = Contour(_n_segments);
}

void ContourSolver::initVars()
{
    std::string names = "abcd";
    std::string axes  = "xyz";

    for (int i = 0; i < _n_segments; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            std::vector<GRBVar> coeff_vars;
            for (int k = 0; k < 3; ++k)
            {
                std::string var_name = std::string(1, names[j]) + axes[k] + std::to_string(i);
                coeff_vars.push_back(_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0,
                                                    GRB_CONTINUOUS, var_name.c_str()));
            }
            _contour[i]._x[j] = GVec({coeff_vars[0], coeff_vars[1], coeff_vars[2]});
        }
    }
}

void ContourSolver::setPolytopeCons()
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
        int cols          = _polytopes[i].cols();
        Eigen::MatrixXd A = _polytopes[i].leftCols(cols - 1);
        Eigen::VectorXd b = -_polytopes[i].rightCols(1);

        Segment segment = _contour.getSegment(i);
        GLEVec p0       = segment.getP0();
        GLEVec p1       = segment.getP1();
        GLEVec p2       = segment.getP2();
        GLEVec p3       = segment.getP3();

        GLEVec Acp0 = A * p0;
        GLEVec Acp1 = A * p1;
        GLEVec Acp2 = A * p2;
        GLEVec Acp3 = A * p3;

        for (int j = 0; j < A.rows(); ++j)
        {
            GRBConstr c0 = _model->addConstr(
                Acp0.x[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p0");

            _polytope_constraints.push_back(c0);

            _polytope_constraints.push_back(_model->addConstr(
                Acp1.x[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p1"));
            _polytope_constraints.push_back(_model->addConstr(
                Acp2.x[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p2"));
            _polytope_constraints.push_back(_model->addConstr(
                Acp3.x[j], GRB_LESS_EQUAL, b(j),
                "Poly" + std::to_string(i) + "_side" + std::to_string(j) + "_p3"));
        }
    }
}

void ContourSolver::setStartCons()
{
    // Remove previous
    for (int i = 0; i < _start_constraints.size(); i++)
    {
        _model->remove(_start_constraints[i]);
    }

    _start_constraints.clear();

    GLEVec pos = _contour[0].getPos(0);
    GLEVec vel = _contour[0].getVel(0);
    GLEVec acc = _contour[0].getAcc(0);

    for (int i = 0; i < _start.pos.size(); ++i)
    {
        _start_constraints.push_back(_model->addConstr(_start.pos[i], GRB_EQUAL, pos[i],
                                                       "Start_pos_axis" + std::to_string(i)));
        _start_constraints.push_back(_model->addConstr(_start.vel[i], GRB_EQUAL, vel[i],
                                                       "Start_vel_axis" + std::to_string(i)));
        _start_constraints.push_back(_model->addConstr(_start.accel[i], GRB_EQUAL, acc[i],
                                                       "Start_acc_axis" + std::to_string(i)));
    }
}

void ContourSolver::setGoalCons()
{
    // Remove previous
    for (int i = 0; i < _goal_constraints.size(); i++)
    {
        _model->remove(_goal_constraints[i]);
    }

    _goal_constraints.clear();

    GLEVec pos = _contour.back().getPos(1);
    GLEVec vel = _contour.back().getVel(1);
    GLEVec acc = _contour.back().getAcc(1);

    for (int i = 0; i < _goal.pos.size(); ++i)
    {
        _goal_constraints.push_back(
            _model->addConstr(std::round(_goal.pos[i] * 1000.0) / 1000.0, GRB_EQUAL, pos[i],
                              "End_pos_axis" + std::to_string(i)));
        /*_goal_constraints.push_back(_model->addConstr(_goal.vel[i], GRB_EQUAL, vel[i],*/
        /*                                              "End_vel_axis" + std::to_string(i)));*/
        /*_goal_constraints.push_back(_model->addConstr(_goal.accel[i], GRB_EQUAL, acc[i],*/
        /*                                              "End_acc_axis" + std::to_string(i)));*/
    }
}

void ContourSolver::setContinuityCons()
{
    // Remove previous
    for (int i = 0; i < _cont_constraints.size(); i++)
    {
        _model->remove(_cont_constraints[i]);
    }

    _cont_constraints.clear();

    for (int i = 0; i < _n_segments - 1; ++i)
    {
        GLEVec pos0 = _contour[i].getPos(1);
        GLEVec vel0 = _contour[i].getVel(1);
        GLEVec acc0 = _contour[i].getAcc(1);

        GLEVec pos1 = _contour[i + 1].getPos(0);
        GLEVec vel1 = _contour[i + 1].getVel(0);
        GLEVec acc1 = _contour[i + 1].getAcc(0);

        for (int j = 0; j < pos0.x.size(); ++j)
        {
            _cont_constraints.push_back(_model->addConstr(pos0.x[j], GRB_EQUAL, pos1.x[j],
                                                          "Cont_pos_axis" + std::to_string(j)));
            _cont_constraints.push_back(_model->addConstr(vel0.x[j], GRB_EQUAL, vel1.x[j],
                                                          "Cont_vel_axis" + std::to_string(j)));
            _cont_constraints.push_back(_model->addConstr(acc0.x[j], GRB_EQUAL, acc1.x[j],
                                                          "Cont_acc_axis" + std::to_string(j)));
        }
    }
}

void ContourSolver::setObjective()
{
    GRBQuadExpr control_cost = 0;

    double gamma    = 1.4;
    double factor   = 1.0;
    double radius_2 = 1.5 * 1.5;

    for (int i = 0; i < _n_segments; i++)
    {
        GLEVec jerk = _contour[i].getJerk(0);

        control_cost += jerk.squaredNorm();
    }
    // m.setObjective(control_cost + final_state_cost + distance_to_JPS_cost, GRB_MINIMIZE);
    _model->setObjective(control_cost, GRB_MINIMIZE);
}

void ContourSolver::setup()
{
    initVars();

    setPolytopeCons();
    setStartCons();
    setGoalCons();
    setContinuityCons();

    setObjective();

    _cb.set_timeout(.15);
    _model->setCallback(&_cb);
    /*_model->set(GRB_IntParam_NumericFocus, 3);*/
}

bool ContourSolver::optimize()
{
    _model->update();
    try
    {
        _model->optimize();
    }
    catch (GRBException &e)
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
        // set dual reductions to 0
        /*_model->reset();*/
        /*_model->feasRelax(1, false, false, true);*/

        /*_model->optimize();*/

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

}  // namespace contour_solver
