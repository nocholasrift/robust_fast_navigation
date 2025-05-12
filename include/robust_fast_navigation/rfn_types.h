#pragma once

#include <gurobi_c++.h>

#include <Eigen/Core>

struct solver_state
{
    Eigen::Vector3d pos   = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel   = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d jerk  = Eigen::Vector3d::Zero();

    double t = 0;
};
typedef struct solver_state rfn_state_t;

struct planner_params
{
    std::string SOLVER;
    double V_MAX;
    double A_MAX;
    double J_MAX;
    double DT;
    double DT_FACTOR_INIT;
    double DT_FACTOR_FINAL;
    double DT_FACTOR_INCREMENT;
    double SOLVER_TRAJ_DT;
    double W_MAX;
    bool FORCE_FINAL_CONSTRAINT;
    int N_SEGMENTS;
    int N_THREADS;
    bool VERBOSE;
    bool USE_MINVO;
    bool PLAN_IN_FREE;
    bool SIMPLIFY_JPS;
    double MAX_SOLVE_TIME;
    double TRIM_DIST;
    int MAX_POLYS;
};
typedef struct planner_params planner_params_t;

class RFNSegment
{
   public:
    RFNSegment()  = default;
    ~RFNSegment() = default;

    Eigen::VectorXd getPos(double t)
    {
        return _x[0] * t * t * t + _x[1] * t * t + _x[2] * t + _x[3];
    }
    Eigen::VectorXd getVel(double t) { return 3 * _x[0] * t * t + 2 * _x[1] * t + _x[2]; }

    Eigen::VectorXd getAcc(double t) { return 6 * _x[0] * t + 2 * _x[1]; }

    Eigen::VectorXd getJerk(double t) { return 6 * _x[0]; }

    Eigen::VectorXd getP0() { return _x[3]; }

    Eigen::VectorXd getP1() { return (_x[2] * _duration + 3 * _x[3]) / 3; }

    Eigen::VectorXd getP2()
    {
        return (_x[1] * _duration * _duration + 2 * _x[2] * _duration + 3 * _x[3]) / 3;
    }

    Eigen::VectorXd getP3() { return getPos(_duration); }

    std::array<Eigen::VectorXd, 4> _x;
    double _duration;
};

class RFNTrajectory
{
   public:
    RFNTrajectory() = default;

    Eigen::VectorXd getPos(double t)
    {
        int m          = getSegment(t);
        double start_t = getSegmentStartTime(m);
        return _segments[m].getPos(t - start_t);
    }

    Eigen::VectorXd getVel(double t)
    {
        int m          = getSegment(t);
        double start_t = getSegmentStartTime(m);
        return _segments[m].getVel(t - start_t);
    }

    Eigen::VectorXd getAcc(double t)
    {
        int m          = getSegment(t);
        double start_t = getSegmentStartTime(m);
        return _segments[m].getAcc(t - start_t);
    }

    Eigen::VectorXd getJerk(double t)
    {
        int m          = getSegment(t);
        double start_t = getSegmentStartTime(m);
        return _segments[m].getJerk(t - start_t);
    }

    int getSegment(double t)
    {
        for (int i = _segments.size() - 1; i >= 0; --i)
        {
            if (t >= getSegmentStartTime(i)) return i;
        }

        return _segments.size() - 1;
    }

    double getSegmentStartTime(int m)
    {
        double t = 0.;
        for (int i = 0; i < m; ++i)
        {
            t += _segments[i]._duration;
        }

        return t;
    }

    std::vector<RFNSegment> _segments;
};

struct GVec;

// seems compiler doesn't like separation of template
// implementation into header and src files
template <typename T>
struct GEVecBase
{
    std::vector<T> x;
    GEVecBase() {}

    GEVecBase(int sz)
    {
        x.reserve(sz);
        for (int i = 0; i < sz; ++i)
        {
            x.push_back(T());
        }
    }

    GEVecBase(std::initializer_list<T> ilist)
    {
        x.reserve(ilist.size());
        for (auto it = ilist.begin(); it != ilist.end(); it++)
        {
            x.push_back(*it);
        }
    }
    GEVecBase operator+(const GVec &rhs) const;

    GEVecBase operator/(double rhs) const
    {
        GEVecBase ret;
        ret.x.reserve(x.size());
        for (int i = 0; i < x.size(); ++i)
        {
            ret.x.push_back(x[i] / rhs);
        }
        return ret;
    }

    T operator[](int i) const { return x[i]; }

    double size() const { return x.size(); }
};

using GQEVec = GEVecBase<GRBQuadExpr>;

struct GLEVec : GEVecBase<GRBLinExpr>
{
    GRBQuadExpr squaredNorm()
    {
        GRBQuadExpr norm = 0;
        for (int i = 0; i < x.size(); ++i)
        {
            norm += x[i] * x[i];
        }
        return norm;
    }
};

struct GVec
{
    std::vector<GRBVar> x;

    GVec() {}
    GVec(std::initializer_list<GRBVar> ilist);

    GLEVec operator+(const GVec &rhs) const;
    GLEVec operator+(const GLEVec &rhs) const;
    GLEVec operator-(const GVec &rhs) const;
    // GLEVec operator*(double rhs) const;
    GLEVec operator/(double rhs) const;

    GRBVar operator[](int i) const { return x[i]; }

    double size() const { return x.size(); }

    friend GLEVec operator*(const GVec &lhs, double rhs);
    friend GLEVec operator*(double lhs, const GVec &rhs);
};

template <typename T1, typename T2>
auto operator+(const GEVecBase<T1> &lhs, const GEVecBase<T2> &rhs)
    -> GEVecBase<decltype(T1() + T2())>
{
    using ReturnType = decltype(T1() + T2());
    if (rhs.x.size() != lhs.x.size() || rhs.x.size() == 0 || lhs.x.size() == 0)
    {
        throw std::runtime_error("GEVecBase: size mismatch " + std::to_string(rhs.x.size()) +
                                 " != " + std::to_string(lhs.x.size()));
    }

    GEVecBase<ReturnType> ret;
    ret.x.reserve(rhs.x.size());
    for (int i = 0; i < rhs.x.size(); ++i)
    {
        ret.x.push_back(lhs.x[i] + rhs.x[i]);
    }
    return ret;
}

template <typename T>
GEVecBase<T> GEVecBase<T>::operator+(const GVec &rhs) const
{
    if (rhs.x.size() != x.size() || rhs.x.size() == 0 || x.size() == 0)
    {
        throw std::runtime_error("GVec: size mismatch " + std::to_string(rhs.x.size()) +
                                 " != " + std::to_string(x.size()));
    }

    GEVecBase ret;
    ret.x.reserve(x.size());
    for (int i = 0; i < x.size(); ++i)
    {
        ret.x.push_back(x[i] + rhs.x[i]);
    }
    return ret;
}

template <typename T1, typename T2>
auto operator-(const GEVecBase<T1> &lhs, const GEVecBase<T2> &rhs)
    -> GEVecBase<decltype(T1() - T2())>
{
    using ReturnType = decltype(T1() - T2());
    if (rhs.x.size() != lhs.x.size() || rhs.x.size() == 0 || lhs.x.size() == 0)
    {
        throw std::runtime_error("GEVecBase: size mismatch " + std::to_string(rhs.x.size()) +
                                 " != " + std::to_string(lhs.x.size()));
    }

    GEVecBase<ReturnType> ret;
    ret.x.reserve(rhs.x.size());
    for (int i = 0; i < rhs.x.size(); ++i)
    {
        ret.x.push_back(lhs.x[i] - rhs.x[i]);
    }
    return ret;
}

template <typename T>
GEVecBase<T> operator*(const GEVecBase<T> &lhs, double rhs)
{
    GEVecBase<T> ret;
    ret.x.reserve(lhs.x.size());
    for (int i = 0; i < lhs.x.size(); ++i)
    {
        ret.x.push_back(lhs.x[i] * rhs);
    }
    return ret;
}

template <typename T>
GEVecBase<T> operator*(double lhs, const GEVecBase<T> &rhs)
{
    return rhs * lhs;
}

template <typename T>
GEVecBase<T> operator*(const Eigen::MatrixXd &lhs, const GEVecBase<T> &rhs)
{
    if (lhs.cols() != rhs.x.size() || rhs.x.size() == 0)
    {
        throw std::runtime_error("GEVecBase: size mismatch " + std::to_string(lhs.cols()) +
                                 " != " + std::to_string(rhs.x.size()));
    }

    GEVecBase<T> ret;
    ret.x.reserve(lhs.rows());
    for (int i = 0; i < lhs.rows(); ++i)
    {
        T expr = 0;
        for (int j = 0; j < lhs.cols(); ++j)
        {
            expr += lhs(i, j) * rhs.x[j];
        }
        ret.x.push_back(expr);
    }

    return ret;
}
