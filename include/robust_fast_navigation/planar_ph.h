#include <robust_fast_navigation/rfn_types.h>

#include <memory>

#include "gurobi_c++.h"

namespace planar_ph
{
/*
 * Implementation of a single planar, quintic Pythagorean-Horograph curve segment
 */

class mycallback : public GRBCallback
{
   public:
    bool should_terminate_;
    mycallback() { should_terminate_ = false; }

    mycallback(double timeout_time)
    {
        this->timeout_time = timeout_time;
        should_terminate_  = false;
    }
    // void abortar();
    void set_timeout(double timeout_time)
    {
        this->timeout_time = timeout_time;
        should_terminate_  = false;
    }

   protected:
    void callback() override
    {  // This function is called periodically along the optimization process.
        //  It is called several times more after terminating the program
        if (where == GRB_CB_MIPNODE)
        {
            double elapsed_time = this->getDoubleInfo(GRB_CB_RUNTIME);
            if (elapsed_time > timeout_time)
            {
                printf("Timeout reached\n");
                should_terminate_ = true;
                GRBCallback::abort();  // This function only does effect when inside the
                                       // function callback() of this class
            }
        }
    }

    double timeout_time;
};

class PHSegment
{
   public:
    PHSegment();
    ~PHSegment();
    GQEVec getPos(double t);
    GQEVec getVel(double t);
    GQEVec getAcc(double t);

    GQEVec getP0();
    GQEVec getP1();
    GQEVec getP2();
    GQEVec getP3();
    GQEVec getP4();
    GQEVec getP5();

    GRBQuadExpr getSigma(int i);
    GRBQuadExpr getS(int i);

    GQEVec getV0();
    GQEVec getV1();
    GQEVec getV2();
    GQEVec getV3();
    GQEVec getV4();

    GQEVec getA0();
    GQEVec getA1();
    GQEVec getA2();
    GQEVec getA3();

    // holds the ctrl points for u and v
    // since PH is quintic, u and v are quadratic
    std::array<GVec, 3> _x;
    Eigen::Vector2d _start;

   private:
    GQEVec evalBezier(const std::vector<GQEVec>& coeffs, double t);

    std::vector<double> _binom5 = {1, 5, 10, 10, 5, 1};
    std::vector<double> _binom4 = {1, 4, 6, 4, 1};
    std::vector<double> _binom3 = {1, 3, 3, 1};
    std::vector<double> _binom2 = {1, 2, 1};
};

class PHContour
{
   public:
    PHContour() = default;
    PHContour(int n_segments);
    PHContour(const std::vector<PHSegment>& segments);
    PHSegment& operator[](int m) { return _segments[m]; }
    PHSegment back() { return _segments.back(); }
    int size() { return _segments.size(); }
    GQEVec getPos(double t);
    GQEVec getVel(double t);
    GQEVec getAcc(double t);
    /*int findSegment(double t);*/

   private:
    std::vector<PHSegment> _segments;
};

class PHContourSolver
{
   public:
    PHContourSolver();
    ~PHContourSolver();
    void setStart(const solver_state& state);
    void setGoal(const solver_state& state);
    void setPolytopes(const std::vector<Eigen::MatrixX3d>& polytopes);
    void setup();
    bool optimize();
    PHContour getContour() { return _contour; }

   private:
    void initVars();

    void setObjective();
    void setStartCons();
    void setGoalCons();
    void setPolytopeCons();
    void setContinuityCons();

    GRBEnv _env;
    std::unique_ptr<GRBModel> _model;

    std::vector<Eigen::MatrixX3d> _polytopes;
    std::vector<GRBQConstr> _polytope_constraints;
    std::vector<GRBQConstr> _start_constraints;
    std::vector<GRBQConstr> _goal_constraints;
    std::vector<GRBQConstr> _cont_constraints;

    solver_state _start;
    solver_state _goal;

    PHContour _contour;

    int _n_segments;

    mycallback _cb;
};

}  // namespace planar_ph
