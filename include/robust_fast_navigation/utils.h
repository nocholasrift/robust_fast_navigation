#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Eigen>
#include <iostream>

Eigen::MatrixX4d expandPoly(const Eigen::MatrixX4d& poly, const double& d){
    Eigen::MatrixX4d ret = poly;

    for(int i = 0; i < ret.rows(); i++){
        Eigen::Vector3f n(ret.row(i)[0], ret.row(i)[1], ret.row(i)[2]);
        Eigen::Vector3f en = n / n.norm();
        
        Eigen::Vector3f p;
        if (n[0] < 1e-6 && n[1] < 1e-6 && n[2] < 1e-6)
            continue;

        if (n[0] >= 1e-6)
            p = Eigen::Vector3f(-ret.row(i)[3]/n[0],0,0);
        else if (n[1] >= 1e-6)
            p = Eigen::Vector3f(0,-ret.row(i)[3]/n[1],0);
        else if (n[2] >= 1e-6)
            p = Eigen::Vector3f(0,0,-ret.row(i)[3]/n[2]);

        ret.row(i)[3] = -n.dot(p) - d*n.dot(en);
    }

    return ret;

}

#endif
