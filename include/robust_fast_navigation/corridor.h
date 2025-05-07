#pragma once

/*#include <robust_fast_navigation/grid_map_util.h>*/
#include <robust_fast_navigation/map_util.h>

#include <chrono>
#include <gcopter/firi.hpp>
#include <gcopter/geo_utils.hpp>
// #include <grid_map_ros/GridMapRosConverter.hpp>
// #include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>

// #include <CGAL/Polygon_2.h>
// #include <CGAL/Segment_2.h>
// #include <CGAL/convex_hull_2.h>
// #include <CGAL/Polygon_set_2.h>
// #include <CGAL/Boolean_set_operations_2.h>
// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>

// typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef Kernel::Point_2                                   Point_2;
// typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
// typedef Kernel::Segment_2                                 Segment_2;
// typedef CGAL::Polygon_set_2<Kernel>                       Polygon_set_2;
// typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;

namespace corridor
{
inline bool convexCover(const std::vector<Eigen::VectorXd> &path,
                        const std::vector<Eigen::VectorXd> &points,
                        const Eigen::Vector3d &lowCorner, const Eigen::Vector3d &highCorner,
                        const double &progress, const double &range,
                        std::vector<Eigen::MatrixX4d> &hpolys, const double eps = 1.0e-6)
{
    if (path.size() < 2)
    {
        std::cout << "path size is too small" << std::endl;
        return false;
    }

    hpolys.clear();
    const int n                    = path.size();
    Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
    bd(0, 0)                       = 1.0;
    bd(1, 0)                       = -1.0;
    bd(2, 1)                       = 1.0;
    bd(3, 1)                       = -1.0;
    bd(4, 2)                       = 1.0;
    bd(5, 2)                       = -1.0;

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
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor> > pc(valid_pc[0].data(),
                                                                            3, valid_pc.size());

        if (!firi::firi(bd, pc, a, b, hp))
        {
            std::cout << "firi failure :(" << std::endl;
            return false;
        }

        if (hpolys.size() != 0)
        {
            const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
            if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                         ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
            {
                if (!firi::firi(bd, pc, a, a, gap, 1))
                {
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

// inline std::vector<Eigen::Vector2d> getOccupied(const costmap_2d::Costmap2D
// &_map)
inline std::vector<Eigen::Vector2d> getOccupied(const map_util::occupancy_grid_t &occ_grid)
{
    std::vector<Eigen::Vector2d> paddedObs;
    // unsigned char *grid = _map.getCharMap();

    double resolution = occ_grid.get_resolution();

    std::vector<int> size = occ_grid.get_size();
    int width             = size[0];
    int height            = size[1];

    for (unsigned int i = 0; i < width; i++)
    {
        for (unsigned int j = 0; j < height; j++)
        {
            /*unsigned int index = occ_grid.cells_to_index(i, j);*/
            /*if (occ_grid.data[index] == costmap_2d::LETHAL_OBSTACLE)*/
            /*    std::cout << "FOUND INFLATED OBSTACLE" << std::endl;*/
            if (occ_grid.is_occupied(i, j, "inflated"))
            {
                unsigned int mx, my;
                double x, y;
                // _map.indexToCells(i, mx, my);
                std::vector<double> coords = occ_grid.map_to_world(i, j);
                x                          = coords[0];
                y                          = coords[1];

                paddedObs.push_back(Eigen::Vector2d(x, y));
            }
        }
    }

    return paddedObs;
}

inline std::vector<Eigen::MatrixX4d> simplifyCorridor(
    const std::vector<Eigen::MatrixX4d> &polys)
{
    int count                         = 0;
    std::vector<Eigen::MatrixX4d> ret = polys;
    std::vector<bool> inds(polys.size());
    inds[0]                = true;
    inds[polys.size() - 1] = true;
    inds[polys.size() - 2] = true;

    bool removed = false;
    // do{
    removed = false;
    for (int i = 0; i < ret.size() - 3;)
    {
        std::cout << i << "/" << ret.size() << std::endl;
        if (geo_utils::overlap(ret[i], ret[i + 1]) && geo_utils::overlap(ret[i], ret[i + 2]))
        {
            ret.erase(ret.begin() + i + 1);
            // std::cout << "^removed" << std::endl;
            removed = true;
        }
        else
        {
            i++;
        }
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
inline bool isInPoly(const Eigen::MatrixX4d &poly, const Eigen::Vector2d &p)
{
    // point needs to be p=(x,y,z,1)
    // in 2D so z = 0
    Eigen::Vector4d pR4(p(0), p(1), 0, 1);
    Eigen::VectorXd res = poly * pR4;

    for (int i = 0; i < res.rows(); i++)
    {
        if (res(i) > 0) return false;
    }
    return true;
}

inline void shrinkPolytopes(std::vector<Eigen::MatrixX4d> &hpolys,
                            const Eigen::MatrixXd &initialPVAJ,
                            const Eigen::MatrixXd &finalPVAJ, double shrinkAmount)
{
    int N            = 5;
    double step_size = shrinkAmount / N;
    int k            = 0;
    for (auto &poly : hpolys)
    {
        int numHalfPlanes = poly.rows();  // Get the number of half-planes (rows in the matrix)

        // Loop through each half-plane
        for (int i = 0; i < numHalfPlanes; ++i)
        {
            // Calculate the magnitude of the normal vector
            Eigen::Vector2d normal = poly.row(i).head<2>();
            normal.normalize();
            for (int j = 0; j < N; ++j)
            {
                // Shrink the polytope by adjusting the offset
                // Subtract shrinkAmount scaled by the magnitude of the normal vector
                poly(i, 3) += step_size;

                // check if overlap with next halfplane
                if (k < hpolys.size() - 1 && !geo_utils::overlap(poly, hpolys[k + 1]))
                {
                    poly(i, 3) -= step_size;
                    break;
                }
                if (k > 0 && !geo_utils::overlap(poly, hpolys[k - 1]))
                {
                    poly(i, 3) -= step_size;
                    break;
                }
                if (k == 0 &&
                    !isInPoly(poly, Eigen::Vector2d(initialPVAJ(0, 0), initialPVAJ(1, 0))))
                {
                    poly(i, 3) -= step_size;
                    break;
                }
                if (k == hpolys.size() - 1 &&
                    !isInPoly(poly, Eigen::Vector2d(finalPVAJ(0, 0), finalPVAJ(1, 0))))
                {
                    poly(i, 3) -= step_size;
                    break;
                }
            }
        }

        k++;
    }
}

/*inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)*/
/*{*/
/*    std::vector<Eigen::MatrixX4d> htemp = hpolys;*/
/*    if (htemp.size() == 1) return;*/
/**/
/*    hpolys.clear();*/
/*    std::deque<int> indices;*/
/*    int M = htemp.size();*/
/**/
/*    indices.push_front(M - 1);*/
/*    int i = M - 1;*/
/*    while (i >= 0)*/
/*    {*/
/*        bool found = false;*/
/*        for (int j = 0; j < i; j++)*/
/*        {*/
/*            if (geo_utils::overlap(htemp[i], htemp[j], 0.01))*/
/*            {*/
/*                indices.push_front(j);*/
/*                i     = j;*/
/*                found = true;*/
/*                break;*/
/*            }*/
/*        }*/
/*        if (!found) --i;*/
/*    }*/
/**/
/*    for (const auto &idx : indices) hpolys.push_back(htemp[idx]);*/
/*}*/

inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
{
    std::vector<Eigen::MatrixX4d> htemp = hpolys;
    if (htemp.size() == 1) return;
    /*if (htemp.size() == 1)*/
    /*{*/
    /*    Eigen::MatrixX4d headPoly = htemp.front();*/
    /*    htemp.insert(htemp.begin(), headPoly);*/
    /*}*/
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

inline bool createCorridorJPS(const std::vector<Eigen::Vector2d> &path,
                              const map_util::occupancy_grid_t &_map,
                              std::vector<Eigen::MatrixX4d> &polys,
                              const Eigen::MatrixXd &initialPVAJ,
                              const Eigen::MatrixXd &finalPVAJ)
{
    static std::vector<Eigen::VectorXd> prev_obs;

    polys.clear();
    std::vector<Eigen::VectorXd> path3d, obs3d;
    for (Eigen::Vector2d p : path)
    {
        path3d.push_back(Eigen::Vector3d(p[0], p[1], 0));
    }

    auto start = std::chrono::high_resolution_clock::now();
    /*for (Eigen::Vector2d ob : getOccupied(_map))*/
    /*{*/
    /*    obs3d.push_back(Eigen::Vector3d(ob[0], ob[1], 0));*/
    /*}*/
    obs3d = _map.get_occupied(3);

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "getOccupied took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;

    if (prev_obs.size() > 0 && obs3d.size() == 0)
    {
        std::cout << "Obstacles in map dissapeared, using previous obstacles\n";
        obs3d = prev_obs;
    }

    prev_obs = obs3d;

    double x          = _map.get_origin()[0];
    double y          = _map.get_origin()[1];
    double resolution = _map.get_resolution();
    double w          = _map.get_size()[0] * resolution;
    double h          = _map.get_size()[1] * resolution;

    bool status = convexCover(path3d, obs3d, Eigen::Vector3d(x, y, -.1),
                              Eigen::Vector3d(x + w, y + h, .1), 7.0, 5.0, polys);

    if (!status) return false;

    shortCut(polys);
    /*shrinkPolytopes(polys, initialPVAJ, finalPVAJ, .2);*/

    std::cout << "POLYS IN CORRIDOR GENERATOR HAS SIZE: " << polys.size() << std::endl;
    /*std::cout << "norm is: " << (polys[0] - polys[1]).norm() << std::endl;*/

    if (!isInPoly(polys[0], Eigen::Vector2d(initialPVAJ(0, 0), initialPVAJ(1, 0))))
    {
        std::cout << "[Corridor] Start was not in poly, adding extra polygon to correct this\n";
        std::vector<Eigen::VectorXd> poses = {initialPVAJ.col(0), initialPVAJ.col(0)};
        std::vector<Eigen::MatrixX4d> tmp;
        status = convexCover(poses, obs3d, Eigen::Vector3d(x, y, -.1),
                             Eigen::Vector3d(x + w, y + h, .1), .1, 5.0, tmp);
        if (status) polys.insert(polys.begin(), tmp[0]);
    }

    if (!isInPoly(polys.back(), Eigen::Vector2d(finalPVAJ(0, 0), finalPVAJ(1, 0))))
    {
        std::cout << "[Corridor] End was not in poly, adding extra polygon to correct this\n";
        std::vector<Eigen::VectorXd> poses = {finalPVAJ.col(0), finalPVAJ.col(0)};
        std::vector<Eigen::MatrixX4d> tmp;
        status = convexCover(poses, obs3d, Eigen::Vector3d(x, y, -.1),
                             Eigen::Vector3d(x + w, y + h, .1), .1, 5.0, tmp);
        if (status) polys.insert(polys.end(), tmp[0]);
    }

    // check if obstacles are inside the corridor
    for (Eigen::Vector3d ob : obs3d)
    {
        if (_map.get_cost(ob(0), ob(1), "obstacles") != costmap_2d::LETHAL_OBSTACLE) continue;

        for (Eigen::MatrixX4d poly : polys)
        {
            if (isInPoly(poly, Eigen::Vector2d(ob(0), ob(1))))
            {
                /*std::cout << "val: " << _map._map.atPosition("obstacles", ob.head(2))*/
                /*          << std::endl;*/
                /*std::cout << "cost at " << ob.transpose() << " is "*/
                /*          << static_cast<int>(_map.get_cost(ob(0), ob(1), "obstacles"))*/
                /*          << std::endl;*/
                /**/
                /*std::cout << "[Corridor] Obstacle in corridor, generation failed!!!"*/
                /*          << std::endl;*/
                return false;
            }
        }
    }

    return true;
}

}  // end namespace corridor
