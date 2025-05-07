#pragma once

#include <robust_fast_navigation/rfn_types.h>

#include <cppad/example/cppad_eigen.hpp>
#include <vector>

#define U0_IND 0
#define U1_IND 1
#define U2_IND 2
#define V0_IND 3
#define V1_IND 4
#define V2_IND 5

/*
 * A templated class for a planar PH curve (Quintic)
 */

// U must be list type
template <typename T, typename U>
class PlanarPH
{
   public:
    using Vec2  = Eigen::Matrix<T, 2, 1>;
    using Vec3  = Eigen::Matrix<T, 3, 1>;
    using VecX  = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using MatX3 = Eigen::Matrix<T, Eigen::Dynamic, 3>;
    /*using Vec2 = CppAD::eigen_vector<T>;*/
    /*using Vec3  = Eigen::Matrix<T, 3, 1>;*/
    /*using VecX  = Eigen::Matrix<T, Eigen::Dynamic, 1>;*/
    /*using MatX3 = Eigen::Matrix<T, Eigen::Dynamic, 3>;*/

    PlanarPH()
    {
        /*start_ = Vec2::Zero();*/
        /*goal_  = Vec2::Zero();*/
        /*poly_  = MatX3::Zero(4, 3);*/
        x_.resize(6);
        for (int i = 0; i < 6; ++i) x_[i] = T(0.);
    }

    template <typename T2, typename = std::enable_if_t<
                               std::is_convertible_v<decltype(std::declval<T2>()[0]), T>>>
    PlanarPH(const T2 &x)
    {
        x_.reserve(6);
        for (int i = 0; i < 6; ++i) x_.emplace_back(T(x[i]));
    }

    template <typename T2, typename = std::enable_if_t<
                               std::is_convertible_v<decltype(std::declval<T2>()[0]), T>>>
    void setX(const T2 &x)
    {
        assert(x.size() == 6 && "PlanarPH: x must be of size 6");
        for (int i = 0; i < 6; ++i) x_[i] = T(x[i]);
    }

    static T evalU(const U &x, T t)
    {
        T u(0.);
        std::array<T, 3> mt_pows_;
        mt_pows_[0] = T(1.);
        for (int i = 1; i < 3; ++i) mt_pows_[i] = mt_pows_[i - 1] * (T(1.) - t);

        T t_pow(1.);
        for (int i = 0; i < 3; i++)
        {
            u += T(binom2_[i]) * x[i] * mt_pows_[2 - i] * t_pow;
            t_pow *= t;
        }

        return u;
    }

    static T evalV(const U &x, T t)
    {
        T v(0.);
        std::array<T, 3> mt_pows_;
        mt_pows_[0] = T(1.);
        for (int i = 1; i < 3; ++i) mt_pows_[i] = mt_pows_[i - 1] * (T(1.) - t);

        T t_pow(1.);
        for (int i = 0; i < 3; i++)
        {
            // add 2 to x index for v coeffs
            v += binom2_[i] * x[2 + i] * mt_pows_[2 - i] * t_pow;
            t_pow *= t;
        }

        return v;
    }

    static T evalDU(const U &x, T t)
    {
        T du =
            T(2) * (x[U1_IND] - x[U0_IND]) * (T(1.) - t) + T(2) * (x[U2_IND] - x[U1_IND]) * t;
        return du;
    }

    static T evalDV(const U &x, T t)
    {
        T dv =
            T(2) * (x[V1_IND] - x[V0_IND]) * (T(1.) - t) + T(2) * (x[V2_IND] - x[V1_IND]) * t;
        return dv;
    }

    static T evalSigma2(const U &x, T t)
    {
        T u = evalU(x, t);
        T v = evalV(x, t);
        return u * u + v * v;
    }

    Vec2 getP1() const { return getP1(x_, start_); }
    Vec2 getP2() const { return getP2(x_, start_); }
    Vec2 getP3() const { return getP3(x_, start_); }
    Vec2 getP4() const { return getP4(x_, start_); }
    Vec2 getP5() const { return getP5(x_, start_); }

    Vec2 getP1(const U &x) const { return getP1(x, start_); }
    Vec2 getP2(const U &x) const { return getP2(x, start_); }
    Vec2 getP3(const U &x) const { return getP3(x, start_); }
    Vec2 getP4(const U &x) const { return getP4(x, start_); }
    Vec2 getP5(const U &x) const { return getP5(x, start_); }

    static Vec2 getP1(const U &x, const Vec2 &start)
    {
        /*Vec2 uv1 = {x[U0_IND] * x[U0_IND] - x[V0_IND] * x[V0_IND], T(2) * x[U0_IND] *
         * x[V0_IND]};*/
        Vec2 p1 = start + (1. / 5) * getV0(x);
        return p1;
    }
    static Vec2 getP2(const U &x, const Vec2 &start)
    {
        /*Vec2 uv2 = {x[U0_IND] * x[U1_IND] - x[V0_IND] * x[V1_IND], x[U0_IND] * x[V1_IND] +
         * x[U1_IND] * x[V0_IND]};*/
        Vec2 p2 = getP1(x, start) + T(1. / 5) * getV1(x);
        return p2;
    }
    static Vec2 getP3(const U &x, const Vec2 &start)
    {
        /*Vec2 uv3a = {x[U1_IND] * x[U1_IND] - x[V1_IND] * x[V1_IND], T(2) * x[U1_IND] *
         * x[V1_IND]};*/
        /*Vec2 uv3b = {x[U0_IND] * x[U2_IND] - x[V0_IND] * x[V2_IND], x[U0_IND] * x[V2_IND] +
         * x[U2_IND]
         * * x[V0_IND]};*/
        Vec2 p3 = getP2(x, start) + T(1. / 5) * getV2(x);
        return p3;
    }
    static Vec2 getP4(const U &x, const Vec2 &start)
    {
        /*Vec2 uv4 = {x[U1_IND] * x[U2_IND] - x[V1_IND] * x[V2_IND], x[U1_IND] * x[V2_IND] +
         * x[U2_IND] * x[V1_IND]};*/
        Vec2 p4 = getP3(x, start) + T(1. / 5) * getV3(x);
        return p4;
    }
    static Vec2 getP5(const U &x, const Vec2 &start)
    {
        /*Vec2 uv5 = {x[U2_IND] * x[U2_IND] - x[V2_IND] * x[V2_IND], T(2) * x[U2_IND] *
         * x[V2_IND]};*/
        Vec2 p5 = getP4(x, start) + T(1. / 5) * getV4(x);
        return p5;
    }

    static Vec2 getPX(const U &x, const Vec2 &start, uint8_t k)
    {
        switch (k)
        {
            case 0:
                return start;
            case 1:
                return getP1(x, start);
            case 2:
                return getP2(x, start);
            case 3:
                return getP3(x, start);
            case 4:
                return getP4(x, start);
            case 5:
                return getP5(x, start);
        }

        std::cerr << "Invalid index for getPX: " << k << std::endl;
        return Vec2::Zero();
    }

    Vec2 getPos(T t) const { return getPos(x_, start_, t); }

    static Vec2 getPos(const U &x, const Vec2 &start, T t)
    {
        std::array<Vec2, 6> ctrl_pts;
        ctrl_pts[0] = start;
        ctrl_pts[1] = getP1(x, start);
        ctrl_pts[2] = getP2(x, start);
        ctrl_pts[3] = getP3(x, start);
        ctrl_pts[4] = getP4(x, start);
        ctrl_pts[5] = getP5(x, start);

        Vec2 pos(0., 0.);
        std::array<T, 6> mt_pows_;
        mt_pows_[0] = T(1.);
        for (int i = 1; i < 6; ++i) mt_pows_[i] = mt_pows_[i - 1] * (T(1.) - t);

        T t_pow(1.);
        for (int i = 0; i <= 5; ++i)
        {
            pos += ctrl_pts[i] * binom5_[i] * mt_pows_[5 - i] * t_pow;
            t_pow *= t;
        }

        /*std::cout << pos[0] << ", " << pos[1] << std::endl;*/

        return pos;
    }

    Vec2 getPosAtLen(T s) const { return getPosAtLen(x_, start_, s); }

    static Vec2 getPosAtLen(const U &x, const Vec2 &start, T s)
    {
        // newton-raphson iteration
        T t = arclenToParam(x, s);

        return getPos(x, start, t);
    }

    static Vec2 getV0(const U &x)
    {
        /*return T(5) * (getP1(x, start) - start);*/
        return {x[U0_IND] * x[U0_IND] - x[V0_IND] * x[V0_IND], T(2) * x[U0_IND] * x[V0_IND]};
    }
    static Vec2 getV1(const U &x)
    {
        /*return T(5) * (getP2(x, start) - getP1(x, start));*/
        return {x[U0_IND] * x[U1_IND] - x[V0_IND] * x[V1_IND],
                x[U0_IND] * x[V1_IND] + x[U1_IND] * x[V0_IND]};
    }
    static Vec2 getV2(const U &x)
    {
        /*return T(5) * (getP3(x, start) - getP2(x, start));*/
        Vec2 uv3a = {x[U1_IND] * x[U1_IND] - x[V1_IND] * x[V1_IND],
                     T(2) * x[U1_IND] * x[V1_IND]};
        Vec2 uv3b = {x[U0_IND] * x[U2_IND] - x[V0_IND] * x[V2_IND],
                     x[U0_IND] * x[V2_IND] + x[U2_IND] * x[V0_IND]};
        return T(2. / 3) * uv3a + T(1. / 3) * uv3b;
    }
    static Vec2 getV3(const U &x)
    {
        /*return T(5) * (getP4(x, start) - getP3(x, start));*/
        return {x[U1_IND] * x[U2_IND] - x[V1_IND] * x[V2_IND],
                x[U1_IND] * x[V2_IND] + x[U2_IND] * x[V1_IND]};
    }
    static Vec2 getV4(const U &x)
    {
        /*return T(5) * (getP5(x, start) - getP4(x, start));*/
        return {x[U2_IND] * x[U2_IND] - x[V2_IND] * x[V2_IND], T(2) * x[U2_IND] * x[V2_IND]};
    }

    static Vec2 getVX(const U &x, uint8_t k)
    {
        switch (k)
        {
            case 0:
                return getV0(x);
            case 1:
                return getV1(x);
            case 2:
                return getV2(x);
            case 3:
                return getV3(x);
            case 4:
                return getV4(x);
        }

        std::cerr << "Invalid index for getPX: " << k << std::endl;
        return Vec2::Zero();
    }

    Vec2 getVel(T t) const { return getVel(x_, t); }

    static Vec2 getVel(const U &x, T t)
    {
        Vec2 pos(0., 0.);
        std::array<T, 5> mt_pows_;
        mt_pows_[0] = T(1.);
        for (int i = 1; i < 5; ++i) mt_pows_[i] = mt_pows_[i - 1] * (T(1.) - t);

        T t_pow(1.);
        for (int i = 0; i <= 4; ++i)
        {
            pos += getVX(x, i) * binom4_[i] * mt_pows_[5 - i] * t_pow;
            t_pow *= t;
        }

        return pos;
    }

    Vec2 getVelAtLen(T s) const { return getVelAtLen(x_, s); }

    static Vec2 getVelAtLen(const U &x, T s)
    {
        // newton-raphson iteration
        T t = arclenToParam(x, s);

        return getVel(x, t);
    }

    T getCurvature(T t) const { return getCurvature(x_, t); }

    static T getCurvature(const U &x, T t)
    {
        T du        = evalDU(x, t);
        T dv        = evalDV(x, t);
        T sigma2    = evalSigma2(x, t);
        T curvature = T(2) * (evalU(x, t) * dv - evalV(x, t) * du) / (sigma2 + T(1e-6));
        return curvature;
    }

    T getPSigma(int8_t k) { return getPSigma(x_, k); }

    static T getPSigma(const U &x, int8_t k)
    {
        uint8_t lb = std::max((int8_t)0, (int8_t)(k - 2));
        uint8_t ub = std::min(k, (int8_t)2);

        double n1Ck = binom4_[k];

        T sigma(0.);
        for (int8_t j = lb; j <= ub; ++j)
        {
            double mCkj = binom2_[k - j];
            double mCj  = binom2_[j];
            sigma += (mCj * mCkj / n1Ck) * (x[j] * x[k - j] + x[3 + j] * x[3 + k - j]);
        }

        return sigma;
    }

    T evalSigma(T t) const { return evalSigma(x_, t); }

    static T evalSigma(const U &x, T t)
    {
        T sigma(0.);
        std::array<T, 5> mt_pows_;
        mt_pows_[0] = T(1.);
        for (int i = 1; i < 5; ++i) mt_pows_[i] = mt_pows_[i - 1] * (T(1.) - t);

        T t_pow(1.);
        for (int i = 0; i <= 4; ++i)
        {
            sigma += getPSigma(x, i) * binom4_[i] * mt_pows_[4 - i] * t_pow;
            t_pow *= t;
        }
        return sigma;
    }

    T evalS(T t) const { return evalS(x_, t); }

    static T evalS(const U &x, T t)
    {
        T s(0.);
        std::array<T, 6> mt_pows_;
        mt_pows_[0] = T(1.);
        for (int i = 1; i < 6; ++i) mt_pows_[i] = mt_pows_[i - 1] * (T(1.) - t);

        T sk(0.);
        T t_pow = t;
        for (int i = 1; i <= 5; ++i)
        {
            /*for (int j = 0; j <= i - 1; ++j) sk += (1. / 5) * getPSigma(x, j);*/
            sk += (1. / 5) * getPSigma(x, i - 1);

            s += sk * binom5_[i] * mt_pows_[5 - i] * t_pow;
            t_pow *= t;
            /*sk = 0.;*/
        }

        return s;
    }

    T getArclen(T t0, T tf) const { return evalS(x_, tf) - evalS(x_, t0); }

    static T getArclen(const U &x, T t0, T tf) { return evalS(x, tf) - evalS(x, t0); }

    static T arclenToParam(const U &x, T s)
    {
        // newton-raphson iteration
        T t = s / getArclen(x, 0, 1);  // assume roughly uniform arc length
        /*std::cout << "t: " << t << std::endl;*/
        if (t > T(1.))
        {
            std::cout << "[PlanarPH] Warning: t > 1, setting to 1" << std::endl;
            t = T(1.);
        }

        for (int i = 0; i < 20; ++i)
        {
            T s_t     = evalS(x, t);
            T sigma_t = evalSigma(x, t);
            t         = t - (s_t - s) / sigma_t;

            if (fabs((s_t - s) / sigma_t) < 1e-3) break;
        }

        return t;
    }

    void setPolyBoundary(const MatX3 &poly) { poly_ = poly; }
    void setStart(const Vec2 &start) { start_ = start; }
    void setGoal(const Vec2 &goal) { goal_ = goal; }

    MatX3 getPolyBoundary() { return poly_; }
    Vec2 getStart() { return start_; }
    Vec2 getGoal() { return goal_; }

   private:
    static constexpr std::array<double, 3> binom2_ = {1, 2, 1};
    static constexpr std::array<double, 5> binom4_ = {1, 4, 6, 4, 1};
    static constexpr std::array<double, 6> binom5_ = {1, 5, 10, 10, 5, 1};

    U x_;

    MatX3 poly_;
    Vec2 start_;
    Vec2 goal_;
};

template <typename T, typename U>
class PlanarTrajectory
{
   public:
    using PPH  = PlanarPH<T, U>;
    using Vec2 = typename PPH::Vec2;

    PlanarTrajectory() = default;
    PlanarTrajectory(const std::vector<PPH> &phs) : segments_(phs) {}

    void resize(int n)
    {
        segments_.clear();
        segments_.reserve(n);
        for (int i = 0; i < n; ++i) segments_.emplace_back();
    }

    void addSegment(const PPH &ph) { segments_.push_back(ph); }

    template <typename T2, typename = std::enable_if_t<
                               std::is_convertible_v<decltype(std::declval<T2>()[0][0]), T>>>
    void setX(const T2 &x)
    {
        for (int i = 0; i < segments_.size() - 1; ++i)
        {
            segments_[i].setX(x[i]);
            segments_[i + 1].setStart(segments_[i].getP5());
        }
        segments_.back().setX(x[segments_.size() - 1]);
    }

    Vec2 getPos(T s) const
    {
        T cum_s(0.);
        for (int i = 0; i < segments_.size(); ++i)
        {
            T s_i = segments_[i].getArclen(0, 1);
            /*std::cout << i << " segment length: " << s_i << std::endl;*/
            if (s < cum_s + s_i) return segments_[i].getPosAtLen(s - cum_s);
            cum_s += s_i;
        }

        std::cout << "[PlanarTrajectory] Warning: s (" << s << ") > total length (" << cum_s
                  << "), returning last point" << std::endl;
        return segments_.back().getPos(1);
    }

    Vec2 getVel(T s) const
    {
        T cum_s(0.);
        for (int i = 0; i < segments_.size(); ++i)
        {
            T s_i = segments_[i].getArclen(0, 1);
            /*std::cout << i << " segment length: " << s_i << std::endl;*/
            if (s < cum_s + s_i) return segments_[i].getVelAtLen(s - cum_s);
            cum_s += s_i;
        }

        return Vec2::Zero();
    }

    Vec2 getAcc(T s) const
    {
        T cum_s(0.);
        for (int i = 0; i < segments_.size(); ++i)
        {
            T s_i = segments_[i].getArclen(0, 1);
            /*std::cout << i << " segment length: " << s_i << std::endl;*/
            if (s < cum_s + s_i) return segments_[i].getPosAtLen(s - cum_s);
            cum_s += s_i;
        }

        return Vec2::Zero();
    }

    T getArcLen() const
    {
        T len = 0;
        for (const auto &seg : segments_)
        {
            len += seg.getArclen(0, 1);
        }

        return len;
    }

    const PPH &getSegment(T s) const
    {
        for (int i = segments_.size() - 1; i >= 0; --i)
        {
            if (s >= segments_[i].getArclen(0, 1)) return segments_[i];
        }
    }

    void setStart(const Vec2 &start)
    {
        if (segments_.size() > 0) segments_[0].setStart(start);
    }

    void setGoal(const Vec2 &goal)
    {
        if (segments_.size() > 0) segments_.back().setGoal(goal);
    }

    PPH &operator[](unsigned int i)
    {
        if (i < 0 || i >= segments_.size())
        {
            std::cerr << "Index out of bounds: " << i << std::endl;
            throw std::out_of_range("Index out of bounds");
        }
        return segments_[i];
    }

    PPH &back() { return segments_.back(); }

    unsigned int numSegments() const { return segments_.size(); }

   private:
    std::vector<PPH> segments_;
};

struct VelContResidual
{
    VelContResidual(const Eigen::Vector2d &start, int n_segments)
        : start_(start), n_segments_(n_segments)
    {
    }

    template <typename T>
    bool operator()(T const *const *xs, T *residuals) const
    {
        /*std::cout << "velocity calculation" << std::endl;*/
        using Vec2      = Eigen::Matrix<T, 2, 1>;
        using PlanarPHd = PlanarPH<T, std::vector<T>>;

        size_t offset = 0;
        for (int i = 0; i < n_segments_ - 1; ++i)
        {
            std::vector<T> x1(xs[0] + i * 6, xs[0] + (i + 1) * 6);
            std::vector<T> x2(xs[0] + (i + 1) * 6, xs[0] + (i + 2) * 6);
            Vec2 v14 = PlanarPHd::getV4(x1);
            Vec2 v20 = PlanarPHd::getV0(x2);

            residuals[offset++] = (v14 - v20)[0];
            residuals[offset++] = (v14 - v20)[1];
        }

        /*std::cout << "vel residual: " << residuals[0] << " " << residuals[1] <<
         * std::endl;*/
        /*std::cout << "\tv14: " << v14[0] << " " << v14[1] << std::endl;*/
        /*std::cout << "\tv20: " << v20[0] << " " << v20[1] << std::endl;*/

        return true;
    }

    int n_segments_;
    Eigen::Vector2d start_;
};

struct AccContResidual
{
    AccContResidual(const Eigen::Vector2d &start, int n_segments)
        : start_(start), n_segments_(n_segments)
    {
    }

    template <typename T>
    bool operator()(T const *const *xs, T *residuals) const
    {
        /*std::cout << "acceleration calculation" << std::endl;*/
        using Vec2      = Eigen::Matrix<T, 2, 1>;
        using PlanarPHd = PlanarPH<T, std::vector<T>>;

        size_t offset = 0;
        for (int i = 0; i < n_segments_ - 1; ++i)
        {
            std::vector<T> x1(xs[0] + i * 6, xs[0] + (i + 1) * 6);
            std::vector<T> x2(xs[0] + (i + 1) * 6, xs[0] + (i + 2) * 6);
            Vec2 a14            = T(4) * (PlanarPHd::getV4(x1) - PlanarPHd::getV3(x1));
            Vec2 a20            = T(4) * (PlanarPHd::getV1(x2) - PlanarPHd::getV0(x2));
            residuals[offset++] = (a14 - a20)[0];
            residuals[offset++] = (a14 - a20)[1];
        }

        /*std::cout << "acc residual: " << residuals[0] << " " << residuals[1] <<
         * std::endl;*/

        return true;
    }

    int n_segments_;
    Eigen::Vector2d start_;
};

struct GoalContResidual
{
    GoalContResidual(const Eigen::Vector2d &start, const Eigen::Vector2d &goal, int n_segments)
        : start_(start), goal_(goal), n_segments_(n_segments)
    {
    }

    template <typename T>
    bool operator()(T const *const *xs, T *residuals) const
    {
        /*std::cout << "goal calculation" << std::endl;*/
        using Vec2      = Eigen::Matrix<T, 2, 1>;
        using PlanarPHd = PlanarPH<T, std::vector<T>>;

        Vec2 start = start_.cast<T>();
        Vec2 goal  = goal_.cast<T>();
        Vec2 end   = PlanarPHd::getP5(std::vector<T>(xs[0], xs[0] + 6), start);
        for (int i = 1; i < n_segments_; ++i)
            end = PlanarPHd::getP5(std::vector<T>(xs[0] + 6 * i, xs[0] + 6 * (i + 1)), end);

        Vec2 diff = (end - goal);

        residuals[0] = diff[0];
        residuals[1] = diff[1];

        /*std::cout << "goal residual: " << diff[0] << " " << diff[1] << std::endl;*/

        return true;
    }

    int n_segments_;
    Eigen::Vector2d start_;
    Eigen::Vector2d goal_;
};

struct SingleSegmentGoalResidual
{
    SingleSegmentGoalResidual(const Eigen::Vector2d &start, const Eigen::Vector2d &goal)
        : start_(start), goal_(goal)
    {
    }

    template <typename T>
    bool operator()(T const *const *xs, T *residuals) const
    {
        using Vec2      = Eigen::Matrix<T, 2, 1>;
        using PlanarPHd = PlanarPH<T, std::vector<T>>;

        // x1 is previous segment
        std::vector<T> x1(xs[0], xs[0] + 6);

        Vec2 start = start_.cast<T>();
        Vec2 goal  = goal_.cast<T>();

        Vec2 diff = PlanarPHd::getP5(x1, start) - goal;

        residuals[0] = diff[0];
        residuals[1] = diff[1];

        /*std::cout << "goal residual: " << diff[0] << " " << diff[1] << std::endl;*/

        return true;
    }

    Eigen::Vector2d start_;
    Eigen::Vector2d goal_;
};

struct ObjectiveFunction
{
    ObjectiveFunction(int n_segments) : n_segments_(n_segments) {}

    template <typename T>
    bool operator()(T const *const *xs, T *residuals) const
    {
        /*std::cout << "objective calculation" << std::endl;*/
        using Vec2      = Eigen::Matrix<T, 2, 1>;
        using PlanarPHd = PlanarPH<T, std::vector<T>>;

        for (int i = 0; i < n_segments_; ++i)
        {
            std::vector<T> x(xs[0] + 6 * i, xs[0] + 6 * (i + 1));

            residuals[i] = T(0.);
            for (int j = 0; j < 10; ++j)
            {
                T curv = PlanarPHd::getCurvature(x, T(j) / 10.0);
                residuals[i] += curv * curv;
            }
        }

        /*std::cout << "objective residual: " << residuals[0] << std::endl;*/

        return true;
    }

    int n_segments_;
    Eigen::Vector2d start_;
    Eigen::Vector2d goal_;
};

struct PolytopeResidualStart
{
    PolytopeResidualStart(const Eigen::Vector2d &start, const Eigen::MatrixX3d &poly)
        : start_(start), poly_(poly)
    {
    }

    template <typename T>
    bool operator()(T const *const *xs, T *residuals) const
    {
        using Vec2      = Eigen::Matrix<T, 2, 1>;
        using PlanarPHd = PlanarPH<T, std::vector<T>>;

        // x1 is previous segment
        std::vector<T> x1(xs[0], xs[0] + 6);
        Vec2 start = start_.cast<T>();

        size_t idx = 0;
        for (int i = 0; i < poly_.rows(); ++i)
        {
            Vec2 ai(T(poly_(i, 0)), T(poly_(i, 1)));
            /*Vec2 ai;*/
            /*ai << T(poly_(i, 0)), T(poly_(i, 1));*/
            T bi             = T(poly_(i, 2));
            residuals[idx++] = ai.dot(start) - bi;
            residuals[idx++] = ai.dot(PlanarPHd::getP1(x1, start)) - bi;
            residuals[idx++] = ai.dot(PlanarPHd::getP2(x1, start)) - bi;
            residuals[idx++] = ai.dot(PlanarPHd::getP3(x1, start)) - bi;
            residuals[idx++] = ai.dot(PlanarPHd::getP4(x1, start)) - bi;
            residuals[idx++] = ai.dot(PlanarPHd::getP5(x1, start)) - bi;
        }

        /*std::cout << "objective residual: " << residuals[0] << std::endl;*/

        return true;
    }

    Eigen::Vector2d start_;
    Eigen::MatrixX3d poly_;
};

struct PolytopeResidualInterior
{
    PolytopeResidualInterior(const Eigen::Vector2d &start,
                             const std::vector<Eigen::MatrixX3d> &polys, int n_segments)
        : start_(start), polys_(polys), n_segments_(n_segments)
    {
    }

    template <typename T>
    bool operator()(T const *const *xs, T *residuals) const
    {
        using Vec2      = Eigen::Matrix<T, 2, 1>;
        using MatX3     = Eigen::Matrix<T, Eigen::Dynamic, 3>;
        using PlanarPHd = PlanarPH<T, std::vector<T>>;

        // x1 is previous segment
        /*std::vector<T> x1(xs[0], xs[0] + 6);*/
        /*std::vector<T> x2(xs[1], xs[1] + 6);*/
        Vec2 start = start_.cast<T>();
        /*Vec2 end   = PlanarPHd::getP5(std::vector<T>(xs[0], xs[0] + 6), start);*/

        size_t idx = 0;
        /*std::cout << "poly calculation" << std::endl;*/
        for (int i = 0; i < n_segments_; ++i)
        {
            std::vector<T> x(xs[0] + i * 6, xs[0] + (i + 1) * 6);

            for (int j = 0; j < polys_[i].rows(); ++j)
            {
                Vec2 ai(T(polys_[i](j, 0)), T(polys_[i](j, 1)));
                T bi = T(polys_[i](j, 2));

                residuals[idx++] = T(0.05) * (ai.dot(start) - bi);
                residuals[idx++] = T(0.05) * (ai.dot(PlanarPHd::getP1(x, start)) - bi);
                residuals[idx++] = T(0.05) * (ai.dot(PlanarPHd::getP2(x, start)) - bi);
                residuals[idx++] = T(0.05) * (ai.dot(PlanarPHd::getP3(x, start)) - bi);
                residuals[idx++] = T(0.05) * (ai.dot(PlanarPHd::getP4(x, start)) - bi);
                residuals[idx++] = T(0.05) * (ai.dot(PlanarPHd::getP5(x, start)) - bi);
            }

            start = PlanarPHd::getP5(x, start);
        }

        /*std::cout << "objective residual: " << residuals[0] << std::endl;*/

        return true;
    }

    int n_segments_;
    Eigen::Vector2d start_;
    std::vector<Eigen::MatrixX3d> polys_;
};
