#include <robust_fast_navigation/rfn_types.h>

GLEVec operator*(const GVec &lhs, double rhs)
{
    GLEVec ret;
    ret.x.reserve(lhs.x.size());
    for (int i = 0; i < lhs.x.size(); ++i)
    {
        ret.x.push_back(rhs * lhs.x[i]);
    }
    return ret;
}

GLEVec operator*(double lhs, const GVec &rhs)
{
    GLEVec ret;
    ret.x.reserve(rhs.x.size());
    for (int i = 0; i < rhs.x.size(); ++i)
    {
        ret.x.push_back(lhs * rhs.x[i]);
    }
    return ret;
}

GVec::GVec(std::initializer_list<GRBVar> ilist)
{
    x.reserve(ilist.size());
    for (auto it = ilist.begin(); it != ilist.end(); it++)
    {
        x.push_back(*it);
    }
}

GLEVec GVec::operator+(const GVec &rhs) const
{
    if (rhs.x.size() != x.size() || rhs.x.size() == 0 || x.size() == 0)
    {
        throw std::runtime_error("GVec: size mismatch " + std::to_string(rhs.x.size()) +
                                 " != " + std::to_string(x.size()));
    }

    GLEVec ret;
    ret.x.reserve(x.size());
    for (int i = 0; i < x.size(); ++i)
    {
        ret.x.push_back(x[i] + rhs.x[i]);
    }
    return ret;
}

GLEVec GVec::operator+(const GLEVec &rhs) const
{
    if (rhs.x.size() != x.size() || rhs.x.size() == 0 || x.size() == 0)
    {
        throw std::runtime_error("GVec: size mismatch " + std::to_string(rhs.x.size()) +
                                 " != " + std::to_string(x.size()));
    }

    GLEVec ret;
    ret.x.reserve(x.size());
    for (int i = 0; i < x.size(); ++i)
    {
        ret.x.push_back(x[i] + rhs.x[i]);
    }
    return ret;
}

GLEVec GVec::operator-(const GVec &rhs) const
{
    if (rhs.x.size() != x.size() || rhs.x.size() == 0 || x.size() == 0)
    {
        throw std::runtime_error("GVec: size mismatch " + std::to_string(rhs.x.size()) +
                                 " != " + std::to_string(x.size()));
    }

    GLEVec ret;
    ret.x.reserve(x.size());
    for (int i = 0; i < x.size(); ++i)
    {
        ret.x.push_back(x[i] + rhs.x[i]);
    }
    return ret;
}

/*GLEVec GVec::operator*(double rhs) const*/
/*{*/
/*    GLEVec ret;*/
/*    ret.x.reserve(x.size());*/
/*    for (int i = 0; i < x.size(); ++i)*/
/*    {*/
/*        ret.x.push_back(x[i] * rhs);*/
/*    }*/
/*    return ret;*/
/*}*/

GLEVec GVec::operator/(double rhs) const
{
    GLEVec ret;
    ret.x.reserve(x.size());
    for (int i = 0; i < x.size(); ++i)
    {
        ret.x.push_back(x[i] / rhs);
    }
    return ret;
}

GQEVec operator*(const GLEVec &lhs, const GLEVec &rhs)
{
    if (lhs.x.size() != rhs.x.size() || lhs.x.size() == 0 || rhs.x.size() == 0)
    {
        throw std::runtime_error("GLEVec: size mismatch " + std::to_string(lhs.x.size()) +
                                 " != " + std::to_string(rhs.x.size()));
    }

    GQEVec ret;
    ret.x.reserve(lhs.x.size());
    for (int i = 0; i < lhs.x.size(); ++i)
    {
        ret.x.push_back(lhs.x[i] * rhs.x[i]);
    }
    return ret;
}
