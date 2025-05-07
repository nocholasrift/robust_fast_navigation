#include <robust_fast_navigation/ph_demo.h>

#include <algorithm>
#include <cppad/core/independent.hpp>
#include <cppad/core/testvector.hpp>
#include <cppad/core/var2par.hpp>
#include <cppad/cppad.hpp>
#include <iostream>
#include <memory>

using CppAD::AD;
using Dvector = CPPAD_TESTVECTOR(double);
using ADVec2  = Eigen::Matrix<AD<double>, 2, 1>;
using ADVec3  = Eigen::Matrix<AD<double>, 3, 1>;
using ADMatX3 = Eigen::Matrix<AD<double>, Eigen::Dynamic, 3>;
using ADVecX  = Eigen::Matrix<AD<double>, Eigen::Dynamic, 1>;

class AtomicSignedDistance : public CppAD::atomic_three<double>
{
    // wrapper for signed distance g(x,y). Input (n) dim = 2. Output (m) dim = 1.
   public:
    AtomicSignedDistance(const Eigen::MatrixXd &sdf, double resolution,
                         const std::vector<double> &origin)
        : CppAD::atomic_three<double>("SignedDistance"),
          sdf_(sdf),
          resolution_(resolution),
          origin_x_(origin[0]),
          origin_y_(origin[1])
    {
    }

    bool for_type(const CppAD::vector<double> &parameter_x,
                  const CppAD::vector<CppAD::ad_type_enum> &type_x,
                  CppAD::vector<CppAD::ad_type_enum> &type_y) override
    {
        assert(parameter_x.size() == type_x.size());
        bool ok = type_x.size() == 2;
        ok &= type_y.size() == 1;

        if (!ok) return false;

        type_y[0] = std::max(type_x[0], type_x[1]);
        return true;
    }

    bool forward(const CppAD::vector<double> &parameter_x,
                 const CppAD::vector<CppAD::ad_type_enum> &type_x, size_t need_y,
                 size_t order_low, size_t order_up, const CppAD::vector<double> &taylor_x,
                 CppAD::vector<double> &taylor_y) override
    {
#ifndef NDEBUG
        size_t n = taylor_x.size();
        size_t m = taylor_y.size();

        assert(type_x.size() == n);
        assert(n == 2);
        assert(m == 1);
        assert(order_low <= order_up);
#endif

        // only handle 0th order
        std::cout << "forward: " << taylor_x[0] << ", " << taylor_x[1] << std::endl;
        if (order_up != 0) return false;

        taylor_y[0] = interpolate(taylor_x[0], taylor_x[1]);
        std::cout << "taylor_y: " << taylor_y[0] << std::endl;
        std::cout << "resolution: " << resolution_ << std::endl;
        /*double scale = 0.;*/
        /*if (taylor_y[0] < 0.5)*/
        /*{*/
        /*    // smooth step function*/
        /*    double t = taylor_y[0] / 0.5;*/
        /*    scale    = 1.0 - t * t * (3.0 - 2.0 * t);*/
        /*}*/
        /*taylor_y[0] = scale * taylor_y[0];*/

        return true;
    }

    bool reverse(const CppAD::vector<double> &parameter_x,
                 const CppAD::vector<CppAD::ad_type_enum> &type_x, size_t order_up,
                 const CppAD::vector<double> &taylor_x, const CppAD::vector<double> &taylor_y,
                 CppAD::vector<double> &partial_x,
                 const CppAD::vector<double> &partial_y) override
    {
        /*std::cout << "reverse" << std::endl;*/
        // only handling 0 order
        assert(order_up <= 1);
        assert(taylor_x.size() == 2);
        assert(taylor_y.size() == 1);
        assert(partial_x.size() == 2);
        assert(partial_y.size() == 1);

        double x  = taylor_x[0];
        double y  = taylor_x[1];
        double py = partial_y[0];

        // convert to map coordinates
        double mx = (x - origin_x_) / resolution_;
        double my = (y - origin_y_) / resolution_;

        int ix = static_cast<int>(mx);
        int iy = static_cast<int>(my);

        ix = std::clamp(ix, (int)0, (int)sdf_.cols() - 2);
        iy = std::clamp(iy, (int)0, (int)sdf_.rows() - 2);

        double dx = (sdf_(iy, ix + 1) - sdf_(iy, ix)) / resolution_;
        double dy = (sdf_(iy + 1, ix) - sdf_(iy, ix)) / resolution_;

        double dist  = sdf_(iy, ix);
        double scale = 0.;
        /*if (dist < 0.5)*/
        /*{*/
        /*    // smooth step function*/
        /*    double t = dist / 0.5;*/
        /*    scale    = 1.0 - t * t * (3.0 - 2.0 * t);*/
        /*}*/

        partial_x[0] = py * dx;  // ∂L/∂x
        partial_x[1] = py * dy;  // ∂L/∂y
        /*partial_x[0] = 1;  // ∂L/∂x*/
        /*partial_x[1] = 1;  // ∂L/∂y*/

        /*std::cout << "partial_x: " << partial_x[0] << ", " << partial_x[1] << std::endl;*/

        /*double dx = mx - ix;*/
        /*double dy = my - iy;*/
        /**/
        /*ix = std::clamp(ix, (int)0, (int)sdf_.cols() - 2);*/
        /*iy = std::clamp(iy, (int)0, (int)sdf_.rows() - 2);*/
        /**/
        /*// Get SDF values at corners*/
        /*double v00 = sdf_(iy, ix);*/
        /*double v10 = sdf_(iy, ix + 1);*/
        /*double v01 = sdf_(iy + 1, ix);*/
        /*double v11 = sdf_(iy + 1, ix + 1);*/
        /**/
        /*// Derivatives of bilinear interpolation w.r.t. gx and gy*/
        /*double df_dgx = (1 - dy) * (v10 - v00) + dy * (v11 - v01);*/
        /*double df_dgy = (1 - dx) * (v01 - v00) + dx * (v11 - v10);*/
        /**/
        /*// Chain rule to convert to ∂f/∂x, ∂f/∂y*/
        /*partial_x[0] = py * df_dgx / resolution_;  // ∂L/∂x*/
        /*partial_x[1] = py * df_dgy / resolution_;  // ∂L/∂y*/

        return true;
    }

    /*private:*/
    Eigen::MatrixXd sdf_;
    double resolution_;
    double origin_x_;
    double origin_y_;

    double interpolate(double x, double y) const
    {
        // convert to map coordinates
        double mx = (x - origin_x_) / resolution_;
        double my = (y - origin_y_) / resolution_;

        int ix = static_cast<int>(mx);
        int iy = static_cast<int>(my);

        ix = std::clamp(ix, (int)0, (int)sdf_.cols() - 2);
        iy = std::clamp(iy, (int)0, (int)sdf_.rows() - 2);

        return sdf_(iy, ix);

        /*double dx = mx - ix;*/
        /*double dy = my - iy;*/
        /**/
        /*ix = std::clamp(ix, (int)0, (int)sdf_.cols() - 2);*/
        /*iy = std::clamp(iy, (int)0, (int)sdf_.rows() - 2);*/
        /**/
        /*// bilinear interpolation*/
        /*double v00 = sdf_(iy, ix);*/
        /*double v10 = sdf_(iy, ix + 1);*/
        /*double v01 = sdf_(iy + 1, ix);*/
        /*double v11 = sdf_(iy + 1, ix + 1);*/
        /**/
        /*return (1 - dx) * (1 - dy) * v00 + dx * (1 - dy) * v10 + (1 - dx) * dy * v01 +*/
        /*       dx * dy * v11;*/
    }
};

class FG_eval
{
   public:
    using ADvector = CPPAD_TESTVECTOR(AD<double>);
    FG_eval(const PlanarTrajectory<AD<double>, ADvector> &traj,
            const std::vector<Eigen::MatrixX3d> &hpolys)
        : traj_(traj)
    {
        setPolys(hpolys);
        vel_const_mult_ = 0.0;
    }

    void setVel(const Eigen::Vector2d &vel)
    {
        vel_            = ADVec2(vel[0], vel[1]);
        vel_const_mult_ = 1.0;
        std::cout << "setting vel: " << vel[0] << ", " << vel[1] << std::endl;
    }

    void setMap(const map_util::occupancy_grid_t &map)
    {
        atomic_sdf_ = std::make_shared<AtomicSignedDistance>(map._sdf, map.get_resolution(),
                                                             map.get_origin());
    }

    void setPolys(const std::vector<Eigen::MatrixX3d> &hpolys)
    {
        hpolys_.clear();
        hpolys_.reserve(hpolys.size());
        for (int i = 0; i < hpolys.size(); ++i)
        {
            ADMatX3 &hpoly = hpolys_.emplace_back(hpolys[i].rows(), 3);
            for (int j = 0; j < hpoly.rows(); ++j)
            {
                hpoly(j, 0) = hpolys[i](j, 0);
                hpoly(j, 1) = hpolys[i](j, 1);
                hpoly(j, 2) = hpolys[i](j, 2);
            }
        }
    }

    void operator()(ADvector &fg, const ADvector &x)
    {
        fg[0]     = 0;
        double dt = 0.10;

        std::cout << "computing objective: " << std::endl;
        for (int i = 0; i < traj_.numSegments(); ++i)
        {
            ADvector xi(6);
            for (int j = 0; j < 6; ++j)
            {
                xi[j] = x[i * 6 + j];
            }

            // curvature
            for (int j = 0; j < 10; ++j)
            {
                AD<double> curvature = traj_[i].getCurvature(xi, j * dt);
                fg[0] += curvature * curvature;
            }
        }

        std::cout << fg[0] << std::endl;

        size_t offset = 1;

        std::cout << "computing C0 continuity: " << std::endl;
        // P1goal = P2start
        ADvector x0(6);
        for (int i = 0; i < 6; ++i)
        {
            x0[i] = x[i];
        }

        /*traj_[0].setStart(traj_[0].getStart());*/
        ADVec2 p5 = traj_[0].getP5(x0);  //, traj_[0].getStart());
        /*CPPAD_TESTVECTOR(AD<double>) input(2), output(1);*/
        /*input[0] = p5[0];*/
        /*input[1] = p5[1];*/
        /*(*atomic_sdf_)(input, output);*/
        /*output[0] = atomic_sdf_->interpolate(CppAD::Value(CppAD::Var2Par(p1[0])),*/
        /*                                     CppAD::Value(CppAD::Var2Par(p1[1])));*/
        /*fg[0] += 10 * (1.0 - output[0]);*/

        for (int i = 0; i < traj_.numSegments() - 1; ++i)
        {
            traj_[i + 1].setStart(p5);

            ADvector xi1(6);
            for (int j = 0; j < 6; ++j)
            {
                xi1[j] = x[(i + 1) * 6 + j];
            }

            p5 = traj_[i + 1].getP5(xi1);
        }

        // P2goal = goal
        ADvector xn(6);
        for (int i = 0; i < 6; ++i)
        {
            xn[i] = x[(traj_.numSegments() - 1) * 6 + i];
        }

        ADVec2 endp5 = traj_.back().getP5(xn);  //, traj_.back().getStart());
        ADVec2 goal  = traj_.back().getGoal();
        ADVec2 diff  = (endp5 - goal);
        fg[offset++] = diff[0];
        fg[offset++] = diff[1];

        std::cout << "goal residual: " << diff[0] << ", " << diff[1] << std::endl;
        std::cout << "p5: " << endp5[0] << ", " << endp5[1] << std::endl;
        std::cout << "goal: " << traj_.back().getGoal()[0] << traj_.back().getGoal()[1]
                  << std::endl;

        /*ADVec2 v0 = traj_[0].getV0(x0);*/
        /*fg[offset++] = vel_const_mult_ * (v0[0] - vel_[0]);*/
        /*fg[offset++] = vel_const_mult_ * (v0[1] - vel_[1]);*/

        // P1'(1) = P2'(0)
        for (int i = 0; i < traj_.numSegments() - 1; ++i)
        {
            ADvector xi(6);
            for (int j = 0; j < 6; ++j)
            {
                xi[j] = x[i * 6 + j];
            }

            ADvector xi1(6);
            for (int j = 0; j < 6; ++j)
            {
                xi1[j] = x[(i + 1) * 6 + j];
            }

            ADVec2 v_const = traj_[i].getV4(xi) - traj_[i + 1].getV0(xi1);
            fg[offset++]   = v_const[0];
            fg[offset++]   = v_const[1];
            std::cout << "vel residual: " << v_const[0] << ", " << v_const[1] << std::endl;
        }

        // P1''(1) = P2''(0)
        for (int i = 0; i < traj_.numSegments() - 1; ++i)
        {
            ADvector xi(6);
            for (int j = 0; j < 6; ++j)
            {
                xi[j] = x[i * 6 + j];
            }

            ADvector xi1(6);
            for (int j = 0; j < 6; ++j)
            {
                xi1[j] = x[(i + 1) * 6 + j];
            }

            ADVec2 a_const = 4 * ((traj_[i].getV4(xi) - traj_[i].getV3(xi)) -
                                  (traj_[i + 1].getV1(xi1) - traj_[i + 1].getV0(xi1)));
            fg[offset++]   = a_const[0];
            fg[offset++]   = a_const[1];

            std::cout << "acc residual: " << a_const[0] << ", " << a_const[1] << std::endl;
        }

        // Poly constraints
        /*ADMatX3 hpoly = hpolys_[0];*/
        /*for (int j = 0; j < hpoly.rows(); ++j)*/
        /*{*/
        /*    ADVec2 ai     = {hpoly(j, 0), hpoly(j, 1)};*/
        /*    AD<double> bi = hpoly(j, 2);*/
        /**/
        /*    AD<double> c1 = ai.dot(traj_[0].getP1(x0)) - bi;*/
        /*    AD<double> c2 = ai.dot(traj_[0].getP2(x0)) - bi;*/
        /*    AD<double> c3 = ai.dot(traj_[0].getP3(x0)) - bi;*/
        /*    AD<double> c4 = ai.dot(traj_[0].getP4(x0)) - bi;*/
        /*    AD<double> c5 = ai.dot(traj_[0].getP5(x0)) - bi;*/
        /**/
        /*    fg[offset++] = c1;*/
        /*    fg[offset++] = c2;*/
        /*    fg[offset++] = c3;*/
        /*    fg[offset++] = c4;*/
        /*    fg[offset++] = c5;*/
        /*}*/

        std::cout << "starting polytopes" << std::endl;
        for (int i = 0; i < traj_.numSegments(); ++i)
        {
            ADvector xi(6);
            for (int j = 0; j < 6; ++j)
            {
                xi[j] = x[i * 6 + j];
            }

            std::cout << "i = " << i << std::endl;
            ADMatX3 hpoly = hpolys_[i];
            for (int j = 0; j < hpoly.rows(); ++j)
            {
                ADVec2 ai     = {hpoly(j, 0), hpoly(j, 1)};
                AD<double> bi = hpoly(j, 2);

                AD<double> c0 = ai.dot(traj_[i].getStart()) - bi;
                AD<double> c1 = ai.dot(traj_[i].getP1(xi)) - bi;
                AD<double> c2 = ai.dot(traj_[i].getP2(xi)) - bi;
                AD<double> c3 = ai.dot(traj_[i].getP3(xi)) - bi;
                AD<double> c4 = ai.dot(traj_[i].getP4(xi)) - bi;
                AD<double> c5 = ai.dot(traj_[i].getP5(xi)) - bi;

                fg[offset++] = c0;
                fg[offset++] = c1;
                fg[offset++] = c2;
                fg[offset++] = c3;
                fg[offset++] = c4;
                fg[offset++] = c5;
            }
        }
        std::cout << "set polytopes" << std::endl;

        // Add polytope repulsion cost
        /*for (int i = 0; i < traj_.numSegments(); ++i)*/
        /*{*/
        /*    ADvector xi(6);*/
        /*    for (int j = 0; j < 6; ++j) xi[j] = x[i * 6 + j];*/
        /**/
        /*    std::vector<ADVec2> points = {traj_[i].getP1(xi), traj_[i].getP2(xi),*/
        /*                                  traj_[i].getP3(xi), traj_[i].getP4(xi),*/
        /*                                  traj_[i].getP5(xi)};*/
        /**/
        /*    for (const auto &poly : hpolys_)*/
        /*    {*/
        /*        for (const auto &p : points)*/
        /*        {*/
        /*            for (int j = 0; j < poly.rows(); ++j)*/
        /*            {*/
        /*                ADVec2 a     = {poly(j, 0), poly(j, 1)};*/
        /*                AD<double> b = poly(j, 2);*/
        /**/
        /*                AD<double> dist_to_face = b - a.dot(p);*/
        /*                AD<double> margin       = 0.1;*/
        /*                AD<double> eps          = 1e-2;*/
        /**/
        /*                // Smooth hinge penalty using softplus*/
        /*                AD<double> slack   = margin - dist_to_face;*/
        /*                AD<double> penalty = CppAD::log(CppAD::exp(slack / eps) + 1);*/
        /*                fg[0] += 0.1 * CppAD::pow(penalty, 2);*/
        /*            }*/
        /*        }*/
        /*    }*/
        /*}*/

        /*ADMatX3 hpolyN = hpolys_.back();*/
        /*for (int j = 0; j < hpolyN.rows(); ++j)*/
        /*{*/
        /*    ADVec2 ai     = {hpolyN(j, 0), hpolyN(j, 1)};*/
        /*    AD<double> bi = hpolyN(j, 2);*/
        /**/
        /*    AD<double> c0 = ai.dot(traj_.back().getStart()) - bi;*/
        /*    AD<double> c1 = ai.dot(traj_.back().getP1(xn)) - bi;*/
        /*    AD<double> c2 = ai.dot(traj_.back().getP2(xn)) - bi;*/
        /*    AD<double> c3 = ai.dot(traj_.back().getP3(xn)) - bi;*/
        /*    AD<double> c4 = ai.dot(traj_.back().getP4(xn)) - bi;*/
        /**/
        /*    fg[offset++] = c0;*/
        /*    fg[offset++] = c1;*/
        /*    fg[offset++] = c2;*/
        /*    fg[offset++] = c3;*/
        /*    fg[offset++] = c4;*/
        /**/
        /*}*/
    }

   private:
    CppAD::ADFun<double> f_;
    std::shared_ptr<AtomicSignedDistance> atomic_sdf_;
    PlanarTrajectory<AD<double>, ADvector> traj_;
    ADVec2 vel_;
    CppAD::AD<double> vel_const_mult_;
    std::vector<ADMatX3> hpolys_;
};

PHSolver::PHSolver() : n_segments_(0)
{
    ph_ = std::make_unique<
        PlanarTrajectory<CppAD::AD<double>, CPPAD_TESTVECTOR(CppAD::AD<double>)>>();

    constrain_vel_ = false;
}

void PHSolver::setStart(const Eigen::Vector2d &start) { start_ = start; }

void PHSolver::setVel(const Eigen::Vector2d &vel)
{
    vel_           = vel;
    constrain_vel_ = true;
}

void PHSolver::setGoal(const Eigen::Vector2d &goal) { goal_ = goal; }

void PHSolver::setSegments(size_t N) { n_segments_ = N; }

void PHSolver::setMap(const map_util::occupancy_grid_t &map)
{
    std::cout << "[PHSolver] Set map!" << std::endl;
    map_ = map;
}

void PHSolver::setPolys(const std::vector<Eigen::MatrixX3d> &hpolys) { hpolys_ = hpolys; }

bool PHSolver::solve(const std::vector<double> &x0)
{
    size_t n_poly_constraints = 0;
    for (const Eigen::MatrixX3d &poly : hpolys_)
    {
        n_poly_constraints += poly.rows();
    }

    n_poly_constraints *= 6;
    /*n_poly_constraints -= 2;*/

    size_t n_vars = 6 * n_segments_;
    /*size_t n_constraints = 6 + n_poly_constraints + 2;*/
    size_t n_constraints = 2 + 4 * (n_segments_ - 1) + n_poly_constraints;

    std::cout << "Solving Problem: " << std::endl;
    std::cout << "\tn_vars: " << n_vars << std::endl;
    std::cout << "\tn_constraints: " << n_constraints << std::endl;

    AtomicSignedDistance atomic_sdf(map_._sdf, map_.get_resolution(), map_.get_origin());

    double x_t = -2.3;
    double y_t = .35;
    CPPAD_TESTVECTOR(AD<double>) ax(2);
    ax[0] = x_t;
    ax[1] = y_t;

    CppAD::Independent(ax);

    CPPAD_TESTVECTOR(AD<double>) ay(1);
    std::cout << "calling atomic sdf" << std::endl;
    atomic_sdf(ax, ay);

    CppAD::ADFun<double> f(ax, ay);
    CPPAD_TESTVECTOR(double) x_val(2);
    x_val[0] = x_t;
    x_val[1] = y_t;

    std::cout << "calling forward" << std::endl;
    CPPAD_TESTVECTOR(double) y_val = f.Forward(0, x_val);
    std::cout << "FUNCTION VAL: " << y_val[0] << std::endl;

    CPPAD_TESTVECTOR(double) dy(1);
    dy[0]                         = 1.0;
    CPPAD_TESTVECTOR(double) grad = f.Reverse(1, dy);
    std::cout << "GRAD: " << grad[0] << ", " << grad[1] << std::endl;

    /*exit(0);*/

    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; ++i)
    {
        vars[i] = x0[i];
        /*vars[i] = 2;*/
        /*std::cout << "x[" << i << "] = " << vars[i] << std::endl;*/
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    for (int i = 0; i < n_vars; ++i)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints - n_poly_constraints; ++i)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
        /*std::cout << "constraints[" << i << "] = " << constraints_lowerbound[i] <<
         * std::endl;*/
    }

    for (int i = 0; i < n_poly_constraints; ++i)
    {
        int idx                     = i + n_constraints - n_poly_constraints;
        constraints_lowerbound[idx] = -1.0e19;
        constraints_upperbound[idx] = 0;
        /*std::cout << "constraints[" << idx << "] = " << constraints_lowerbound[idx]*/
        /*          << std::endl;*/
    }

    std::cout << "finished constraints, setting up ph" << std::endl;

    // use separate object for solving in case failure
    PlanarTrajectory<AD<double>, CPPAD_TESTVECTOR(CppAD::AD<double>)> ph;
    ph.resize(n_segments_);
    std::cout << "done resizing" << std::endl;
    ph.setStart(start_.cast<AD<double>>());
    std::cout << "done setting start" << std::endl;
    ph.setGoal(goal_.cast<AD<double>>());
    std::cout << "done setting goal" << std::endl;

    FG_eval fg_eval(ph, hpolys_);
    fg_eval.setVel(vel_);
    fg_eval.setMap(map_);
    /*fg_eval.setSegments(n_segments_);*/
    /*fg_eval.setStart(start_);*/
    /*fg_eval.setGoal(goal_);*/

    /*if (hpolys_.size() > 0)*/
    /*{*/
    /*    fg_eval.setPolys(hpolys_);*/
    /*}*/

    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true forward\n";
    options += "Sparse  true reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          .25\n";
    options += "Numeric tol 1e-4\n";  // looser tolerance
    options += "Numeric dual_inf_tol 1e-4\n";
    options += "Numeric acceptable_tol 1e-4\n";  // let IPOPT accept looser solution

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    /*solution.x = vars;*/

    /*solution.zl.resize(n_vars);*/
    /*solution.zu.resize(n_vars);*/
    /*solution.lambda.resize(n_constraints);*/
    /**/
    /*for (int i = 0; i < n_vars; ++i)*/
    /*{*/
    /*    solution.zl[i] = 1e-8;*/
    /*    solution.zu[i] = 1e-8;*/
    /*}*/
    /**/
    /*for (int i = 0; i < n_constraints; ++i)*/
    /*{*/
    /*    solution.lambda[i] = 0.0;*/
    /*}*/

    // std::cerr << "solving now" << std::endl;
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound,
                                          constraints_lowerbound, constraints_upperbound,
                                          fg_eval, solution);

    // Check some of the solution values
    bool ok = true;
    ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success ||
           solution.status == CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point ||
           solution.status == CppAD::ipopt::solve_result<Dvector>::feasible_point_found);

    if (!ok)
    {
        std::cerr << "Error: " << solution.status << std::endl;
        return false;
    }
    else
    {
        x_.clear();
        x_.reserve(n_segments_);
        CPPAD_TESTVECTOR(AD<double>) x(6);

        for (int i = 0; i < n_segments_; ++i)
        {
            x_.emplace_back(6);

            std::cout << "u = [";
            for (int j = 0; j < 3; ++j)
            {
                std::cout << solution.x[i * 6 + j];
                if (j < 2)
                {
                    std::cout << ", ";
                }
                x_[i][j] = solution.x[i * 6 + j];
                x[j]     = solution.x[i * 6 + j];
            }
            std::cout << "]" << std::endl;

            std::cout << "v = [";
            for (int j = 3; j < 6; ++j)
            {
                std::cout << solution.x[i * 6 + j];
                if (j < 5)
                {
                    std::cout << ", ";
                }
                x_[i][j] = solution.x[i * 6 + j];
                x[j]     = solution.x[i * 6 + j];
            }
            std::cout << "]" << std::endl;

            /*(*ph_)[i].setX(x);*/
        }

        ph_->resize(n_segments_);
        ph_->setStart(start_.cast<AD<double>>());
        ph_->setGoal(goal_.cast<AD<double>>());
        ph_->setX(x_);

        constrain_vel_ = false;

        /*std::cout << "start for segment 0: " << (*ph_)[0].getStart()[0] << ", "*/
        /*          << (*ph_)[0].getStart()[1] << std::endl;*/
        /*for (int i = 1; i < n_segments_; ++i)*/
        /*{*/
        /*    (*ph_)[i].setStart((*ph_)[i - 1].getP5());*/
        /*    std::cout << "start for segment " << i << ": " << (*ph_)[i].getStart()[0] << ",
         * "*/
        /*              << (*ph_)[i].getStart()[1] << std::endl;*/
        /*}*/
    }

    return ok;
}

std::vector<double> PHSolver::warm_start()
{
    double *xs = new double[6 * n_segments_];
    for (int i = 0; i < 6 * n_segments_; ++i)
    {
        xs[i] = 1.5;
    }

    ceres::Problem problem;

    /***************************
     * OBJECTIVE FUNCTION
     ***************************/

    auto *objcost = new ceres::DynamicAutoDiffCostFunction<ObjectiveFunction, 1>(
        new ObjectiveFunction(n_segments_));
    objcost->AddParameterBlock(6 * n_segments_);
    objcost->SetNumResiduals(n_segments_);

    problem.AddResidualBlock(objcost, nullptr, &xs[0]);

    std::cout << "Registered objective" << std::endl;

    /***************************
     * C1 CONTINUITY
     ***************************/

    if (n_segments_ > 1)
    {
        auto *velcost = new ceres::DynamicAutoDiffCostFunction<VelContResidual, 4>(
            new VelContResidual(start_, n_segments_));

        velcost->AddParameterBlock(6 * n_segments_);
        velcost->SetNumResiduals(2 * (n_segments_ - 1));

        problem.AddResidualBlock(velcost, nullptr, &xs[0]);

        std::cout << "Registered C1 continuity" << std::endl;

        /***************************
         * C2 CONTINUITY
         ***************************/

        auto *acccost = new ceres::DynamicAutoDiffCostFunction<AccContResidual, 4>(
            new AccContResidual(start_, n_segments_));

        acccost->AddParameterBlock(6 * n_segments_);
        acccost->SetNumResiduals(2 * (n_segments_ - 1));

        problem.AddResidualBlock(acccost, nullptr, &xs[0]);

        std::cout << "Registered C2 continuity" << std::endl;
    }

    /***************************
     * GOAL CONSTRAINT
     ***************************/

    auto *goalcost = new ceres::DynamicAutoDiffCostFunction<GoalContResidual, 4>(
        new GoalContResidual(start_, goal_, n_segments_));

    goalcost->AddParameterBlock(6 * n_segments_);
    goalcost->SetNumResiduals(2);

    problem.AddResidualBlock(goalcost, nullptr, &xs[0]);

    std::cout << "Registered goal" << std::endl;

    /***************************
     * POLYTOPE CONSTRAINTS
     ***************************/

    /*auto *x1polycost = new ceres::DynamicAutoDiffCostFunction<PolytopeResidualStart, 4>(*/
    /*    new PolytopeResidualStart(start, polys[0]));*/
    /**/
    /*x1polycost->AddParameterBlock(6);*/
    /*x1polycost->SetNumResiduals(6 * polys[0].rows());*/
    /**/
    /*problem.AddResidualBlock(x1polycost, nullptr, xs[0]);*/

    // flatten xs

    if (hpolys_.size() > 0)
    {
        size_t poly_constrs = 0;
        for (int i = 0; i < hpolys_.size(); ++i)
        {
            poly_constrs += hpolys_[i].rows();
        }
        auto *x2polycost = new ceres::DynamicAutoDiffCostFunction<PolytopeResidualInterior, 4>(
            new PolytopeResidualInterior(start_, hpolys_, n_segments_));

        // Add two parameter blocks, each length 6
        x2polycost->AddParameterBlock(6 * n_segments_);
        x2polycost->SetNumResiduals(6 * poly_constrs);
        problem.AddResidualBlock(x2polycost, nullptr, &xs[0]);
        std::cout << "Registered polys" << std::endl;
    }

    /***************************
     * SOLVE
     ***************************/

    ceres::Solver::Options options;
    ceres::StringToMinimizerType("trust_region", &options.minimizer_type);

    options.max_num_iterations           = 100;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // print out coeffs
    std::vector<double> ret;
    /*std::cout << "u = [" << xs[0] << ", " << xs[1] << ", " << xs[2] << "]" << std::endl;*/
    /*std::cout << "v = [" << xs[3] << ", " << xs[4] << ", " << xs[5] << "]" << std::endl;*/
    /**/
    /*std::cout << "u = [" << xs[6] << ", " << xs[7] << ", " << xs[8] << "]" << std::endl;*/
    /*std::cout << "v = [" << xs[9] << ", " << xs[10] << ", " << xs[11] << "]" << std::endl;*/

    for (int i = 0; i < 6 * n_segments_; ++i)
    {
        ret.push_back(xs[i]);
    }

    return ret;
}

const PlanarTrajectory<AD<double>, CPPAD_TESTVECTOR(CppAD::AD<double>)> &PHSolver::getTraj()
    const
{
    return *ph_;
}
