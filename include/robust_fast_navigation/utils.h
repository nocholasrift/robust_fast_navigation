#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include <costmap_2d/costmap_2d_ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <decomp_basis/data_type.h>


/**********************************************************************
    This function takes in a polygon and shifts all hyperplanes outward
    along the normals by a distance d.

    Inputs:
      - poly: polygon to expand
      - d: distance to move all hyperplanes

    Returns:
      - Expanded polygon
***********************************************************************/
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

/**********************************************************************
    This function checks if a given point is inside a polygon.

    Inputs:
      - poly: polygon under consideration
      - p: Point to test

    Returns:
      - boolean whether point is in polygon or not.
***********************************************************************/
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

/**********************************************************************
    This function returns the distance of a point to a polygon

    Inputs:
      - poly: polygon under consideration
      - p: Point to test

    Returns:
      - distance of point to polygon

    TODO: Maybe add feature to make distance negative if point is
    inside polygon?
***********************************************************************/
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

/**********************************************************************
    This function determines if a given trajectory is fully contained
    within a given corridor. The function also passes in a threshold
    variable, where the trajectory will still be considered "inside" 
    the corridor if no point is too far outside.

    Inputs:
      - traj: trajectory under consideration
      - hpolys: polygonal corridor used to test against trajectory
      - thresh: allowed tolerance for trajectory to be outisde corridor

    Returns:
      - boolean whether trajectory is in or out of corridor.
***********************************************************************/
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

/**********************************************************************
    This function determines if a given trajectory overlaps the lethal
    obstacles in a costmap. 

    Inputs:
      - traj: trajectory under consideration
      - map: costmap used for testing

    Returns:
      - boolean whether trajectory overlaps obstacles or not.
***********************************************************************/
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

bool testMap(unsigned char val, unsigned char test_val){

    unsigned char true_test_val = test_val;

    if (test_val == costmap_2d::FREE_SPACE)
        return val >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

    return val != test_val;
}

// following bresenham / raycast method from https://docs.ros.org/en/api/costmap_2d/html/costmap__2d_8h_source.html
void bresenham(const costmap_2d::Costmap2D& _map, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
    int offset_b, unsigned int offset, unsigned int max_range, unsigned int& term, 
    unsigned int test_val = costmap_2d::FREE_SPACE){
    
    unsigned int end = std::min(max_range, abs_da);
    unsigned int mx, my;
    for (unsigned int i = 0; i < end; ++i) {
        offset += offset_a;
        error_b += abs_db;
        
        _map.indexToCells(offset, mx, my);
        if (testMap(_map.getCost(mx, my), test_val)){
            break;
        }

        if ((unsigned int)error_b >= abs_da) {
            offset += offset_b;
            error_b -= abs_da;
        }

        _map.indexToCells(offset, mx, my);
        if (testMap(_map.getCost(mx, my), test_val)){
            break;
        }
    }
    
    term = offset;
}

void raycast(const costmap_2d::Costmap2D& _map, unsigned int sx, unsigned int sy, 
    unsigned int ex, unsigned int ey, double &x, double &y, 
    unsigned int test_val = costmap_2d::FREE_SPACE, unsigned int max_range = 1e6){
    
    unsigned int size_x = _map.getSizeInCellsX();

    int dx = ex - sx;
    int dy = ey - sy;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    
    int offset_dx = dx > 0 ? 1 : -1;
    int offset_dy = (dy > 0 ? 1 : -1) * size_x;

    unsigned int offset = sy * size_x + sx;

    double dist = hypot(dx, dy);
    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_range / dist);

    unsigned int term; 
    if (abs_dx >= abs_dy){
        int error_y = abs_dx / 2;
        bresenham(_map, abs_dx, abs_dy, error_y, offset_dx, offset_dy, 
                offset, (unsigned int)(scale * abs_dx), term, test_val);
    } else{
        int error_x = abs_dy / 2;
        bresenham(_map, abs_dy, abs_dx, error_x, offset_dy, offset_dx,
                offset, (unsigned int)(scale * abs_dy), term, test_val);
    }

    // convert costmap index to world coordinates
    unsigned int mx, my;
    _map.indexToCells(term, mx, my);
    _map.mapToWorld(mx, my, x, y);
}

std::vector<Eigen::Vector2d> getJPSInFree(const std::vector<Eigen::Vector2d>& path, const costmap_2d::Costmap2D& map){

    std::vector<Eigen::Vector2d> ret;

    if (path.size() < 2)
        return ret;

    // check if start point is in unknown space, happens when robot has
    // yet to move in map.
    int count = 0;
    unsigned int initX, initY, endX, endY;
    map.worldToMap(path[0][0], path[0][1], initX, initY);
    map.worldToMap(path[1][0], path[1][1], endX, endY);

    while (map.getCost(initX, initY) == costmap_2d::NO_INFORMATION && count++ < 10){
        double x,y;
        
        map.mapToWorld(initX, initY, x, y);
        // std::cout << "looking at (" << x << "," << y << ") " << count << std::endl;
        raycast(map, initX, initY, endX, endY, x, y, costmap_2d::NO_INFORMATION);
        // std::cout << "raycast stopped at (" << x << "," << y << ") " << count << std::endl;
        // std::cout << "end is " << path[1].transpose() << std::endl;
        if (!map.worldToMap(x,y, initX, initY))
            break;

        // std::cout << "map[" << initX << "," << initY <<  "]= " << 
            // (int) map.getCost(initX, initY) << std::endl;
    } 

    if (map.getCost(initX, initY) == costmap_2d::NO_INFORMATION || 
        (initX == endX && initY == endY)){
        std::cerr << "COULDN'T FIND FREE SPACE CLOSE TO ROBOT ALONG JPS" << std::endl;
        return ret;
    }

    double x,y;
    unsigned int sx, sy, ex, ey;

    map.mapToWorld(initX, initY, x, y);

    ret.push_back(Eigen::Vector2d(x, y));
    std::cout << "pushing back " << ret[0].transpose() << std::endl;

    for(int i = 0; i < path.size()-1; i++){
        
        if(!map.worldToMap(path[i+1][0], path[i+1][1], ex, ey) || 
           !map.worldToMap(ret[i][0], ret[i][1], sx, sy)){

            std::cerr << "WARNING: jps path goes outside map" << std::endl;
            return ret;
        }
        
        std::cout << "start " <<  sx << " " << sy << std::endl;
        std::cout << "end " << ex << " " << ey << std::endl;

        raycast(map, sx, sy, ex, ey, x, y);

        Eigen::Vector2d p(x,y);
        if ((path[i+1]-p).norm() > 1e-6){
            std::cout << "hit free space, adding " << p.transpose() << std::endl;
            ret.push_back(p);
            break;
        }

        std::cout << "pushing back " << path[i+1].transpose() << std::endl;
        ret.push_back(path[i+1]);
    }

    return ret;
}

// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
bool lineSphereIntersection( 
    const Eigen::Vector2d& A, 
    const Eigen::Vector2d& B,
    const Eigen::Vector2d& center, 
    const double& r, 
    Eigen::Vector2d& p){
    
    Eigen::Vector2d d = B-A;
    Eigen::Vector2d f = A-center;

    double a = d.dot(B);
    double b = 2*f.dot(d);
    double c = f.dot(f) - r*r;

    double discriminant = b*b-4*a*c;
    if (discriminant < 0)
        return false;

    discriminant = sqrt(discriminant);

    double t1 = (-b-discriminant)/(2*a);
    double t2 = (-b+discriminant)/(2*a);

    if (t1 >= 0 && t1 <= 1){
        p = A+t1*d;
        return true;
    }

    if (t2 >= 0 && t2 <= 1){
        p = A+t2*d;
        return true;
    }

    return false;
}

bool getJPSIntersectionWithSphere(  
    const std::vector<Eigen::Vector2d>& path, 
    std::vector<Eigen::Vector2d>& modified_path, 
    const Eigen::Vector2d& center,
    const double& r,
    Eigen::Vector2d& p){

    if (path.size() == 0)
        return false;

    modified_path.push_back(path[0]);

    for(int i = 0; i < path.size()-1; i++){

        if(lineSphereIntersection(path[i], path[i+1], center, r, p)){
            modified_path.push_back(p);
            return true;
        }

        modified_path.push_back(path[i+1]);
    }

    return false;
    
}

#endif
