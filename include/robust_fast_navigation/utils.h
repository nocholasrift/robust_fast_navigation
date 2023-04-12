#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include <costmap_2d/costmap_2d_ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <decomp_basis/data_type.h>

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

bool isInPoly(const Eigen::MatrixX4d& poly, const Eigen::Vector2d& p){
    // point needs to be p=(x,y,z,1)
    // in 2D so z = 0
    Eigen::Vector4d pR4(p(0), p(1), 0, 1);
    Eigen::VectorXd res = poly*pR4;

    for (int i = 0; i < res.rows(); i++){
        if (res(i) > 0)
            return false;
    }
    return true;

}

double dist2Poly(const Eigen::Vector2d& point, const Eigen::MatrixX4d& poly){

    double dist = 100000.;

    for(int i = 0; i < poly.rows(); i++){
        Eigen::Vector4d plane = poly.row(i);
        double d = fabs(plane[0]*point[0]+plane[1]*point[1]+plane[3]);
        double n = sqrt(plane[0]*plane[0]+plane[1]*plane[1]);
        double plane_dist = d / n;

        if (plane_dist < dist)
            dist = plane_dist;
    }

    return dist;
}

bool isTrajOutsidePolys(const trajectory_msgs::JointTrajectory& traj, const std::vector<Eigen::MatrixX4d>& hpolys, double thresh){
    
    int last_poly_idx = 0;


    for(const trajectory_msgs::JointTrajectoryPoint& pt : traj.points){

        Eigen::Vector2d traj_p(pt.positions[0], pt.positions[1]);
        bool in_corridor = false;
        double max_dist = -1.;

        for(int p = last_poly_idx; p < hpolys.size(); p++){
            bool in_corridor = isInPoly(hpolys[p], traj_p);
            if (in_corridor){
                last_poly_idx = p;
                break;
            } 

            max_dist = std::max(max_dist,  dist2Poly(traj_p, hpolys[p]));
        }
        
        if (max_dist > thresh)
            return true;

    }

    return false;
}

bool isTrajOverlappingObs(const trajectory_msgs::JointTrajectory& traj, const costmap_2d::Costmap2D& map){

    for(const trajectory_msgs::JointTrajectoryPoint& pt : traj.points){

        unsigned int mx, my;
        if (map.worldToMap(pt.positions[0], pt.positions[1], mx, my)){

            if (map.getCost(mx, my) == costmap_2d::LETHAL_OBSTACLE) // || costmap_2d::)
                return true;
        }
    }

    return false;
}



#endif
