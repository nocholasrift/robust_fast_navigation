#pragma once

// #include <endian.h>
#include <gurobi_c++.h>
#include <robust_fast_navigation/rfn_types.h>

#include <Eigen/Core>
#include <array>
#include <memory>

namespace contour_solver {

class mycallback : public GRBCallback {
public:
  bool should_terminate_;
  mycallback() { should_terminate_ = false; }

  mycallback(double timeout_time) {
    this->timeout_time = timeout_time;
    should_terminate_ = false;
  }
  // void abortar();
  void set_timeout(double timeout_time) {
    this->timeout_time = timeout_time;
    should_terminate_ = false;
  }

protected:
  void callback() override { // This function is called periodically along the
                             // optimization process.
    //  It is called several times more after terminating the program
    if (where == GRB_CB_MIPNODE) {
      double elapsed_time = this->getDoubleInfo(GRB_CB_RUNTIME);
      if (elapsed_time > timeout_time) {
        printf("Timeout reached\n");
        should_terminate_ = true;
        GRBCallback::abort(); // This function only does effect when inside the
                              // function callback() of this class
      }
    }
  }

  double timeout_time;
};

class Segment {
public:
  Segment();
  ~Segment();

  GLEVec getPos(double t);
  GLEVec getVel(double t);
  GLEVec getAcc(double t);
  GLEVec getJerk(double t);

  GLEVec getP0();
  GLEVec getP1();
  GLEVec getP2();
  GLEVec getP3();

  std::array<GVec, 4> _x;
};

class Contour {
public:
  Contour() = default;
  Contour(int n_segments);
  Contour(const std::vector<Segment> &segments);

  Segment &operator[](int m) { return _segments[m]; }

  Segment back() { return _segments.back(); }
  int size() { return _segments.size(); }

  GLEVec getPos(double t);
  GLEVec getVel(double t);
  GLEVec getAcc(double t);
  GLEVec getJerk(double t);

  int findSegment(double t);

  Segment getSegment(int m);

private:
  std::vector<Segment> _segments;
};

class ContourSolver {
public:
  ContourSolver();
  ~ContourSolver();

  void setStart(const solver_state &state);
  void setGoal(const solver_state &state);
  void setPolytopes(const std::vector<Eigen::MatrixX4d> &polytopes);

  Contour getContour() { return _contour; }

  void setup();
  bool optimize();

private:
  void initVars();

  void setStartCons();
  void setGoalCons();
  void setPolytopeCons();
  void setContinuityCons();

  void setObjective();

  // std::unique_ptr<GRBEnv> _env;
  GRBEnv _env;
  std::unique_ptr<GRBModel> _model;

  std::vector<Eigen::MatrixX4d> _polytopes;
  std::vector<GRBConstr> _polytope_constraints;
  std::vector<GRBConstr> _start_constraints;
  std::vector<GRBConstr> _goal_constraints;
  std::vector<GRBConstr> _cont_constraints;

  solver_state _start;
  solver_state _goal;

  Contour _contour;

  int _n_segments;

  mycallback _cb;
};
} // namespace contour_solver
