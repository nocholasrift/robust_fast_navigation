#include <robust_fast_navigation/ph_demo.h>

#include <chrono>
#include <iostream>
#include <memory>

using CppAD::AD;
using Dvector = CPPAD_TESTVECTOR(double);
using ADVec2  = Eigen::Matrix<AD<double>, 2, 1>;
using ADVec3  = Eigen::Matrix<AD<double>, 3, 1>;
using ADMatX3 = Eigen::Matrix<AD<double>, Eigen::Dynamic, 3>;
using ADVecX  = Eigen::Matrix<AD<double>, Eigen::Dynamic, 1>;

class FG_eval
{
   public:
    using ADvector = CPPAD_TESTVECTOR(AD<double>);
    FG_eval(const PlanarTrajectory<AD<double>, ADvector> &traj,
            const std::vector<Eigen::MatrixX3d> &hpolys)
        : traj_(traj)
    {
        setPolys(hpolys);
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

            for (int j = 0; j < 10; ++j)
            {
                AD<double> curvature = traj_[i].getCurvature(xi, j * dt);
                fg[0] += curvature * curvature;
            }
        }

        std::cout << fg[0] << std::endl;

        size_t offset = 1;

        // P1goal = P2start
        ADvector x0(6);
        for (int i = 0; i < 6; ++i)
        {
            x0[i] = x[i];
        }
        ADVec2 p5 = traj_[0].getP5(x0);
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

        ADVec2 endp5 = traj_.back().getP5(xn);
        ADVec2 goal  = traj_.back().getGoal();
        ADVec2 diff  = (endp5 - goal);
        fg[offset++] = diff[0];
        fg[offset++] = diff[1];

        std::cout << "goal residual: " << diff[0] << diff[1] << std::endl;
        std::cout << "goal: " << traj_.back().getGoal()[0] << traj_.back().getGoal()[1]
                  << std::endl;

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

            ADVec2 a_const = 4 * (traj_[i].getV4(xi) - traj_[i].getV3(xi) -
                                  (traj_[i + 1].getV1(xi1) - traj_[i + 1].getV0(xi1)));
            fg[offset++]   = a_const[0];
            fg[offset++]   = a_const[1];

            std::cout << "acc residual: " << a_const[0] << ", " << a_const[1] << std::endl;
        }

        // Poly constraints
        ADMatX3 hpoly = hpolys_[0];
        for (int j = 0; j < hpoly.rows(); ++j)
        {
            ADVec2 ai     = {hpoly(j, 0), hpoly(j, 1)};
            AD<double> bi = hpoly(j, 2);

            AD<double> c1 = ai.dot(traj_[0].getP1(x0)) - bi;
            AD<double> c2 = ai.dot(traj_[0].getP2(x0)) - bi;
            AD<double> c3 = ai.dot(traj_[0].getP3(x0)) - bi;
            AD<double> c4 = ai.dot(traj_[0].getP4(x0)) - bi;
            AD<double> c5 = ai.dot(traj_[0].getP5(x0)) - bi;

            fg[offset++] = c1;
            fg[offset++] = c2;
            fg[offset++] = c3;
            fg[offset++] = c4;
            fg[offset++] = c5;

            /*std::cout << "poly 0 residual: " << c1 << ", " << c2 << ", " << c3 << ", " << c4*/
            /*          << ", " << c5 << std::endl;*/
        }

        for (int i = 1; i < traj_.numSegments() - 1; ++i)
        {
            ADvector xi(6);
            for (int j = 0; j < 6; ++j)
            {
                xi[j] = x[i * 6 + j];
            }

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

                /*std::cout << "poly " << i << " residual: " << c1 << ", " << c2 << ", " << c3*/
                /*          << ", " << c4 << ", " << c5 << std::endl;*/
            }
        }

        ADMatX3 hpolyN = hpolys_.back();
        for (int j = 0; j < hpolyN.rows(); ++j)
        {
            ADVec2 ai     = {hpolyN(j, 0), hpolyN(j, 1)};
            AD<double> bi = hpolyN(j, 2);

            AD<double> c0 = ai.dot(traj_.back().getStart()) - bi;
            AD<double> c1 = ai.dot(traj_.back().getP1(xn)) - bi;
            AD<double> c2 = ai.dot(traj_.back().getP2(xn)) - bi;
            AD<double> c3 = ai.dot(traj_.back().getP3(xn)) - bi;
            AD<double> c4 = ai.dot(traj_.back().getP4(xn)) - bi;

            fg[offset++] = c0;
            fg[offset++] = c1;
            fg[offset++] = c2;
            fg[offset++] = c3;
            fg[offset++] = c4;

            /*std::cout << "polyN residual: " << c1 << ", " << c2 << ", " << c3 << ", " << c4*/
            /*          << ", " << std::endl;*/
        }
    }

   private:
    PlanarTrajectory<AD<double>, ADvector> traj_;
    std::vector<ADMatX3> hpolys_;
};

PHSolver::PHSolver() : n_segments_(0)
{
    ph_ = std::make_unique<
        PlanarTrajectory<CppAD::AD<double>, CPPAD_TESTVECTOR(CppAD::AD<double>)>>();
}

void PHSolver::setStart(const Eigen::Vector2d &start) { start_ = start; }

void PHSolver::setGoal(const Eigen::Vector2d &goal) { goal_ = goal; }

void PHSolver::setSegments(size_t N) { n_segments_ = N; }

void PHSolver::setPolys(const std::vector<Eigen::MatrixX3d> &hpolys) { hpolys_ = hpolys; }

bool PHSolver::solve(const std::vector<double> &x0)
{
    size_t n_poly_constraints = 0;
    for (const Eigen::MatrixX3d &poly : hpolys_)
    {
        n_poly_constraints += poly.rows();
    }

    n_poly_constraints *= 6;
    n_poly_constraints -= 2;

    size_t n_vars        = 6 * n_segments_;
    size_t n_constraints = 6 + n_poly_constraints;

    std::cout << "Solving Problem: " << std::endl;
    std::cout << "\tn_vars: " << n_vars << std::endl;
    std::cout << "\tn_constraints: " << n_constraints << std::endl;

    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; ++i)
    {
        vars[i] = x0[i];
        /*vars[i] = 2;*/
        std::cout << "x[" << i << "] = " << vars[i] << std::endl;
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

// int main()
// {
//     using namespace std::chrono;
//
//     Eigen::Vector2d start(0, 0);
//     Eigen::Vector2d goal(8, 0);
//
//     PHSolver solver;
//
//     std::vector<Eigen::MatrixX3d> hpolys;
//     Eigen::MatrixX3d hpoly1(4, 3);
//     hpoly1.row(0) << -1, 0, 1;
//     hpoly1.row(1) << 1, 0, 1;
//     hpoly1.row(2) << 0, -1, 1;
//     hpoly1.row(3) << 0, 1, 6;
//     /*hpoly1.row(0) << -1, 0, 1e3;*/
//     /*hpoly1.row(1) << 1, 0, 1e3;*/
//     /*hpoly1.row(2) << 0, -1, 1e3;*/
//     /*hpoly1.row(3) << 0, 1, 1e3;*/
//
//     Eigen::MatrixX3d hpoly2(4, 3);
//     hpoly2.row(0) << -1, 0, 1;
//     hpoly2.row(1) << 1, 0, 6;
//     hpoly2.row(2) << 0, -1, -4;
//     hpoly2.row(3) << 0, 1, 6;
//     /*hpoly2.row(0) << -1, 0, 1e3;*/
//     /*hpoly2.row(1) << 1, 0, 1e3;*/
//     /*hpoly2.row(2) << 0, -1, 1e3;*/
//     /*hpoly2.row(3) << 0, 1, 1e3;*/
//
//     Eigen::MatrixX3d hpoly3(4, 3);
//     hpoly3.row(0) << 1, 0, 9;
//     hpoly3.row(1) << -1, 0, -5.5;
//     hpoly3.row(2) << 0, -1, 15;
//     hpoly3.row(3) << 0, 1, 15;
//     /*hpoly3.row(0) << -1, 0, 1e3;*/
//     /*hpoly3.row(1) << 1, 0, 1e3;*/
//     /*hpoly3.row(2) << 0, -1, 1e3;*/
//     /*hpoly3.row(3) << 0, 1, 1e3;*/
//
//     hpolys.push_back(hpoly1);
//     hpolys.push_back(hpoly2);
//     hpolys.push_back(hpoly3);
//
//     size_t M = 3;
//
//     solver.setPolys(hpolys);
//     solver.setSegments(M);
//     solver.setStart(start);
//     solver.setGoal(goal);
//
//     std::cout << hpoly1.block(0, 0, 4, 2) * start - hpoly1.col(2) << std::endl;
//     std::cout << hpoly2.block(0, 0, 4, 2) * goal - hpoly2.col(2) << std::endl;
//     std::cout << hpoly3.block(0, 0, 4, 2) * goal - hpoly3.col(2) << std::endl;
//
//     auto st                = high_resolution_clock::now();
//     std::vector<double> x0 = solver.warm_start();
//     bool status            = solver.solve(x0);
//     auto stop              = high_resolution_clock::now();
//     auto duration          = duration_cast<milliseconds>(stop - st);
//     std::cout << "Duration: " << duration.count() << " ms" << std::endl;
//
//     if (!status)
//     {
//         std::cerr << "Solver failed" << std::endl;
//         return 1;
//     }
//     else
//     {
//         std::cout << "Solver succeeded" << std::endl;
//         std::cout << "Total arclength is: " << solver.getTraj().getArcLen() << std::endl;
//         double s = 12;
//         std::cout << solver.getTraj().getPos(s)[0] << ", " << solver.getTraj().getPos(s)[1]
//                   << std::endl;
//         /*std::cout << "pos at"*/
//     }
//     return 0;
// }
