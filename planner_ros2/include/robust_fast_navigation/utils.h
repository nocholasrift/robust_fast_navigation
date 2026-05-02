#ifndef ROBUST_FAST_NAVIGATION_UTILS_H
#define ROBUST_FAST_NAVIGATION_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>

#include "gcopter/geo_utils.hpp"

namespace ros2_utils{

inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys,
                              const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &meshPub, 
                              const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &edgePub,
                              const rclcpp::Time &stamp,
                              const std::string &frame_id = "map",
                              bool highlight = false)
{
    if (hPolys.empty()) return;

    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    for (size_t id = 0; id < hPolys.size(); id++)
    {
        oldTris = mesh;
        Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
        geo_utils::enumerateVs(hPolys[id], vPoly);

        quickhull::QuickHull<double> tinyQH;
        const auto polyHull   = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
        const auto &idxBuffer = polyHull.getIndexBuffer();
        int hNum              = idxBuffer.size() / 3;

        curTris.resize(3, hNum * 3);
        for (int i = 0; i < hNum * 3; i++)
        {
            curTris.col(i) = vPoly.col(idxBuffer[i]);
        }
        mesh.resize(3, oldTris.cols() + curTris.cols());
        mesh.leftCols(oldTris.cols())  = oldTris;
        mesh.rightCols(curTris.cols()) = curTris;
    }

    visualization_msgs::msg::Marker meshMarker, edgeMarker;

    meshMarker.id                 = 0;
    meshMarker.header.stamp       = stamp;
    meshMarker.header.frame_id    = frame_id;
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action             = visualization_msgs::msg::Marker::ADD;
    meshMarker.type               = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    meshMarker.ns                 = "mesh";

    meshMarker.color.r = 0.675f;
    meshMarker.color.g = 0.988f;
    meshMarker.color.b = 0.851f;
    meshMarker.color.a = 0.15f;

    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker      = meshMarker;
    edgeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edgeMarker.ns   = "edge";

    if (highlight)
    {
        edgeMarker.color.r = 1.0f;
        edgeMarker.color.g = 0.0f;
        edgeMarker.color.b = 0.0f;
    }
    else
    {
        edgeMarker.color.r = 0.365f;
        edgeMarker.color.g = 0.851f;
        edgeMarker.color.b = 0.757f;
    }
    edgeMarker.color.a = 1.00f;
    edgeMarker.scale.x = 0.02;

    geometry_msgs::msg::Point point;
    int ptnum = static_cast<int>(mesh.cols());

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

    meshPub->publish(meshMarker);
    edgePub->publish(edgeMarker);
}
    
}

#endif