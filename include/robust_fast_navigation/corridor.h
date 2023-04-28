#ifndef CORRIDOR_H
#define CORRIDOR_H

#include <iostream>

#include <gcopter/firi.hpp>
#include <gcopter/geo_utils.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/Marker.h>

#include <decomp_util/seed_decomp.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_geometry/geometric_utils.h>

#include <robust_fast_navigation/utils.h>

namespace corridor{

    inline bool convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            if (!firi::firi(bd, pc, a, b, hp)){
                std::cout << "firi failure :(" << std::endl;
                return false;
            }

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    if (!firi::firi(bd, pc, a, a, gap, 1)){
                        std::cout << "firi failure :(" << std::endl;
                        return false;
                    }
                        
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }

        return true;
    }

    inline Eigen::MatrixX4d getHyperPlanes(const Polyhedron<2>& poly, const Eigen::Vector2d& seed){
        
        vec_E<Hyperplane<2> > planes = poly.vs_;

        auto vertices = cal_vertices(poly);
        vertices.push_back(vertices[0]);

        Eigen::MatrixX4d hPoly(vertices.size()+1, 4);

        for(int i = 0; i < vertices.size()-1; i++){

            // hyperplane from vertex to vertex
            Vecf<2> v1 = vertices[i];
            Vecf<2> v2 = vertices[i+1];
            Eigen::Vector4d hypEq;
            if (fabs(v1(0)-v2(0)) < 1e-6)
                hypEq << 1,0,0,-v1(0);
            else{
                double m = (v2(1)-v1(1))/(v2(0)-v1(0));
                hypEq << -m,1,0,-v1(1)+m*v1(0);
            }
            
            if (hypEq(0)*seed(0)+hypEq(1)*seed(1)+hypEq(2)*.5+hypEq(3) > 0)
                hypEq *= -1;

            hPoly.row(i) = hypEq;
        }

        Eigen::Vector4d hypEqZ0(0,0,1,-.5);
        if (hypEqZ0(0)*seed(0)+hypEqZ0(1)*seed(1)+hypEqZ0(2)*.5+ hypEqZ0(3) > 0)
            hypEqZ0 *= -1;

        Eigen::Vector4d hypEqZ1(0,0,-1,-.5);
        if (hypEqZ1(0)*seed(0)+hypEqZ1(1)*seed(1)+hypEqZ1(2)*.5+ hypEqZ1(3) > 0)
            hypEqZ1 *= -1;

        hPoly.row(vertices.size()-1) = hypEqZ0;
        hPoly.row(vertices.size()) = hypEqZ1;

        return hPoly;
    }

    inline vec_Vec2f getPaddedScan(const costmap_2d::Costmap2D& _map, 
                                    double seedX, double seedY, const vec_Vec2f& _obs){

        int offset = 1000000;

        double x, y;
        unsigned int sx, sy, ex, ey;

        vec_Vec2f paddedObs;

        if (!_map.worldToMap(seedX, seedY, sx, sy)){
            ROS_ERROR("FATAL: Odometry reading is NOT inside costmap!");
            // exit(-1);
            return paddedObs;
        }

        for(Vec2f ob : _obs){
            if (!_map.worldToMap(ob[0], ob[1], ex, ey))
                continue;

            raycast(_map, sx,sy,ex,ey,x,y);
            paddedObs.push_back(Vec2f(x,y));
         
        }

        return paddedObs;
    }

    inline vec_Vec2f getOccupied(const costmap_2d::Costmap2D& _map){

        vec_Vec2f paddedObs;
        unsigned char* grid = _map.getCharMap();

        double resolution = _map.getResolution();
        int width = _map.getSizeInCellsX();
        int height = _map.getSizeInCellsY();

        for(int i = 0; i < width*height; i++){

            if (grid[i] == costmap_2d::LETHAL_OBSTACLE || 
                grid[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                unsigned int mx, my;
                double x, y;
                _map.indexToCells(i, mx, my);
                _map.mapToWorld(mx, my, x, y);

                paddedObs.push_back(Vec2f(x,y));
            }

        }

        return paddedObs;
    }

    inline vec_Vec2f getLethal(const costmap_2d::Costmap2D& _map){

        vec_Vec2f paddedObs;
        unsigned char* grid = _map.getCharMap();

        double resolution = _map.getResolution();
        int width = _map.getSizeInCellsX();
        int height = _map.getSizeInCellsY();

        for(int i = 0; i < width*height; i++){

            if(grid[i] == costmap_2d::LETHAL_OBSTACLE){
                unsigned int mx, my;
                double x, y;
                _map.indexToCells(i, mx, my);
                _map.mapToWorld(mx, my, x, y);

                paddedObs.push_back(Vec2f(x,y));
            }

        }

        return paddedObs;
    }


    inline Eigen::MatrixX4d genPoly(const costmap_2d::Costmap2D& _map, 
                                    double x, double y, const vec_Vec2f& _obs){
        SeedDecomp2D decomp(Vec2f(x, y));
        // decomp.set_obs(_obs);
        decomp.set_obs(getPaddedScan(_map, x, y, _obs));
        decomp.set_local_bbox(Vec2f(2,2));
        decomp.dilate(.1);

        auto poly = decomp.get_polyhedron();
    
        return getHyperPlanes(poly, Eigen::Vector2d(x,y));
    }

    inline std::vector<Eigen::MatrixX4d> genPolyJPS(const costmap_2d::Costmap2D& _map, 
                                    const std::vector<Eigen::Vector2d>& path,
                                    const vec_Vec2f& _obs){

        std::vector<Eigen::MatrixX4d> polys;
        EllipsoidDecomp2D decomp;
        // decomp.set_obs(_obs);
        // double x = ((p1+p2)/2)(0);
        // double y = ((p1+p2)/2)(1);
        decomp.set_obs(_obs);
        decomp.set_local_bbox(Vec2f(2,2));

        vec_Vecf<2> p;
        for(auto point : path)
            p.push_back(point);
        
        decomp.dilate(p);

        auto result = decomp.get_polyhedrons();
    
        for(int i = 0; i < result.size()-1; i++)
            polys.push_back(getHyperPlanes(result[i], (path[i]+path[i+1])/2));

        return polys;
    }

    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys,
                                    const ros::Publisher& meshPub,
                                    const ros::Publisher& edgePub){

        // Due to the fact that H-representation cannot be directly visualized
        // We first conduct vertex enumeration of them, then apply quickhull
        // to obtain triangle meshs of polyhedra
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
        for (size_t id = 0; id < hPolys.size(); id++)
        {
            oldTris = mesh;
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
            geo_utils::enumerateVs(expandPoly(hPolys[id],.05), vPoly);

            quickhull::QuickHull<double> tinyQH;
            const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
            const auto &idxBuffer = polyHull.getIndexBuffer();
            int hNum = idxBuffer.size() / 3;

            curTris.resize(3, hNum * 3);
            for (int i = 0; i < hNum * 3; i++)
            {
                curTris.col(i) = vPoly.col(idxBuffer[i]);
            }
            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;
        }

        // RVIZ support tris for visualization
        visualization_msgs::Marker meshMarker, edgeMarker;

        meshMarker.id = 0;
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = "map";
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.action = visualization_msgs::Marker::ADD;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";
        meshMarker.color.r = 0.675;
        meshMarker.color.g = 0.988;
        meshMarker.color.b = .851;
        meshMarker.color.a = 0.15;
        meshMarker.scale.x = 1.0;
        meshMarker.scale.y = 1.0;
        meshMarker.scale.z = 1.0;

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.365;
        edgeMarker.color.g = 0.851;
        edgeMarker.color.b = 0.757;
        edgeMarker.color.a = 1.00;
        edgeMarker.scale.x = 0.02;

        geometry_msgs::Point point;

        int ptnum = mesh.cols();

        for (int i = 0; i < ptnum; i++)
        {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i);
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }

        meshPub.publish(meshMarker);
        edgePub.publish(edgeMarker);

        return;
    }


    inline std::vector<Eigen::MatrixX4d> 
    simplifyCorridor(const std::vector<Eigen::MatrixX4d>& polys){

        int count = 0;
        std::vector<Eigen::MatrixX4d> ret = polys;
        std::vector<bool> inds (polys.size());
        inds[0] = true;
        inds[polys.size()-1] = true;
        inds[polys.size()-2] = true;

        bool removed = false;
        // do{
            removed = false;
            for(int i = 0; i < ret.size()-3; ){
                
                // int j = i;
                // while(geo_utils::overlap(polys[i], polys[j+1]) &&
                //             geo_utils::overlap(polys[i], polys[j+2])){

                //     polys.erase(polys.begin()+j);
                // }

                // inds[i+1] = !(geo_utils::overlap(polys[i], polys[i+1]) &&
                //             geo_utils::overlap(polys[i], polys[i+2]));
                //     // ret.erase(ret.begin()+i+1);
                // if(geo_utils::overlap(polys[i], polys[i+1]) &&
                //             geo_utils::overlap(polys[i], polys[i+2]))
                //             std::cout << i+1 << " is not neccessary" << std::endl;
                std::cout << i << "/" << ret.size() << std::endl;
                if(geo_utils::overlap(ret[i], ret[i+1]) &&
                            geo_utils::overlap(ret[i], ret[i+2])){
                    ret.erase(ret.begin()+i+1);
                    // std::cout << "^removed" << std::endl;
                    removed = true;
                } else{
                    i++;
                }
            }

        // } while(removed);

        // for(int i = 0; i < polys.size(); i++){

        //     if(inds[i])
        //         ret.push_back(polys[i]);
        // }

        return ret;

    }


    inline std::vector<Eigen::MatrixX4d> createCorridor(
        const std::vector<Eigen::Vector2d>& path, const costmap_2d::Costmap2D& _map,
        const vec_Vec2f& _obs){
        
        int max_iters = 1000;

        std::vector<Eigen::MatrixX4d> polys;
        Eigen::Vector2d seed(path[0](0), path[0](1));
        polys.push_back(genPoly(_map, seed(0), seed(1), _obs));

        Eigen::Vector2d goal = path.back();

        int i = 0;
        int lastStopped = 0;
        while (i < max_iters && !isInPoly(polys.back(), goal)){

            Eigen::Vector2d newSeed;
            for (int k = lastStopped; k < path.size(); k++){
                if (!isInPoly(polys.back(), path[k]) || k == path.size()-1){
                    lastStopped = k;
                    newSeed = path[k];
                    break;
                }
            }

            polys.push_back(genPoly(_map, newSeed(0), newSeed(1), _obs));
            i++;
        }

        return polys;
    }

    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

    inline bool createCorridorJPS(
        const std::vector<Eigen::Vector2d>& path, const costmap_2d::Costmap2D& _map,
        const vec_Vec2f& _obs, std::vector<Eigen::MatrixX4d>& polys){

        polys.clear();
        std::vector<Eigen::Vector3d> path3d, obs3d;
        for(Eigen::Vector2d p : path){
            path3d.push_back(Eigen::Vector3d(p[0], p[1], 0));
        }

        // for(Vec2f ob : getPaddedScan(_map, path[0][0], path[0][1], _obs)){
        //     obs3d.push_back(Eigen::Vector3d(ob[0], ob[1], 0));
        // }

        for(Vec2f ob : getOccupied(_map)){
            obs3d.push_back(Eigen::Vector3d(ob[0], ob[1], 0));
        }

        double x = _map.getOriginX();
        double y = _map.getOriginY();
        double w = _map.getSizeInMetersX();
        double h = _map.getSizeInMetersY();

        // ROS_INFO("(%.2f, %.2f) --> (%.2f, %.2f)", x, y, x+w, y+h);
        // exit(0);
        bool status = convexCover(path3d, obs3d, Eigen::Vector3d(x,y,-.1), 
            Eigen::Vector3d(x+w,y+h,.1),7.0, 5.0, polys);

        if (!status)
            return false;

        shortCut(polys);
        
        return true;

        // for(int i = 0; i < path.size()-1; i++){
        //     polys.push_back(genPolyJPS(_map, path[i], path[i+1], _obs));
        // }
        
        // std::vector<Eigen::MatrixX4d> ret = simplifyCorridor(polys);
        // return ret;
        // shortCut(polys);
        // return polys;
    }

    inline bool createCorridorBRS(
        const std::vector<Eigen::Vector2d>& path, const nav_msgs::OccupancyGrid& mapMsg,
        const std::vector<Eigen::Vector2d>& obs, std::vector<Eigen::MatrixX4d>& polys){

        polys.clear();
        std::vector<Eigen::Vector3d> path3d, obs3d;
        for(Eigen::Vector2d p : path){
            path3d.push_back(Eigen::Vector3d(p[0], p[1], 0));
        }

        for(Eigen::Vector2d ob : obs)
            obs3d.push_back(Eigen::Vector3d(ob[0], ob[1], 0));

        double x = mapMsg.info.origin.position.x;
        double y = mapMsg.info.origin.position.y;
        double w = mapMsg.info.width;
        double h = mapMsg.info.height;

        bool status = convexCover(path3d, obs3d, Eigen::Vector3d(x,y,-.1), 
            Eigen::Vector3d(x+w,y+h,.1),7.0, 5.0, polys);

        if (!status)
            return false;

        shortCut(polys);
        
        return true;

        // for(int i = 0; i < path.size()-1; i++){
        //     polys.push_back(genPolyJPS(_map, path[i], path[i+1], _obs));
        // }
        
        // std::vector<Eigen::MatrixX4d> ret = simplifyCorridor(polys);
        // return ret;
        // shortCut(polys);
        // return polys;
    }

}

#endif
