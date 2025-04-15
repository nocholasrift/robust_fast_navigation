#pragma once

#include <costmap_2d/cost_values.h>

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "grid_map_cv/GridMapCvConverter.hpp"
#include "opencv2/imgproc.hpp"

namespace map_util
{
// struct mimicing nav_msgs::OccupancyGrid
class OccupancyGrid
{
   public:
    grid_map::GridMap _map;
    std::string _obstacle_layer;

    std::vector<unsigned char> occupied_values;

    OccupancyGrid() {}

    OccupancyGrid(const grid_map::GridMap &grid_map, const std::string &obs_layer,
                  const std::vector<unsigned char> &ov, double radius)
    {
        _map            = grid_map;
        _obstacle_layer = obs_layer;
        occupied_values = ov;
        inflate_obstacles(radius, "inflated");
    }

    // define these functions with vectors so we can pybind them more easily
    std::vector<unsigned int> world_to_map(double x, double y) const
    {
        grid_map::Position p(x, y);
        grid_map::Index indices;
        if (!_map.getIndex(p, indices))
            throw std::invalid_argument("[world_to_map] x or y is not within map bounds");

        return {static_cast<unsigned int>(indices[0]), static_cast<unsigned int>(indices[1])};
    }

    std::vector<double> map_to_world(unsigned int mx, unsigned int my) const
    {
        grid_map::Index idx(mx, my);
        grid_map::Position p;

        if (!_map.getPosition(idx, p))
            throw std::invalid_argument(
                "[map_to_world] mx or my is greater than width or height");

        return {p[0], p[1]};
    }

    std::vector<unsigned char> get_data(const std::string &layer)
    {
        std::vector<unsigned char> data;
        data.resize(_map.getSize()(0) * _map.getSize()(1));

        int i = 0;
        for (grid_map::GridMapIterator it(_map); !it.isPastEnd(); ++it)
        {
            grid_map::Index idx = *it;
            data[i++]           = _map.at(layer, idx);
        }

        return data;
    }

    double get_resolution() const { return _map.getResolution(); }

    std::vector<double> get_origin() const
    {
        grid_map::Position p = _map.getPosition();
        grid_map::Position o = p - grid_map::Position(0.5 * _map.getLength());
        std::cout << _map.getLength() << std::endl;
        return {o[0], o[1]};
    }

    std::vector<int> get_size() const { return {_map.getSize()[0], _map.getSize()[1]}; }

    unsigned char get_cost(double x, double y, const std::string &layer) const
    {
        grid_map::Position p(x, y);
        float val = _map.atPosition(layer, p);

        unsigned char ret = costmap_2d::FREE_SPACE;
        if (val >= 99)
            ret = costmap_2d::LETHAL_OBSTACLE;
        else if (val >= 98)
            ret = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

        return ret;
    }

    unsigned char get_cost(unsigned int mx, unsigned int my, const std::string &layer) const
    {
        grid_map::Index idx(mx, my);
        float val = _map.at(layer, idx);

        unsigned char ret = costmap_2d::FREE_SPACE;
        if (val >= 99)
            ret = costmap_2d::LETHAL_OBSTACLE;
        else if (val >= 98)
            ret = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

        return ret;
    }

    bool is_occupied(double x, double y, const std::string &layer) const
    {
        return std::find(occupied_values.begin(), occupied_values.end(),
                         get_cost(x, y, layer)) != occupied_values.end();
    }

    bool is_occupied(unsigned int mx, unsigned int my, const std::string &layer) const
    {
        return std::find(occupied_values.begin(), occupied_values.end(),
                         get_cost(mx, my, layer)) != occupied_values.end();
    }

    void inflate_obstacles(double radius, const std::string &inflated_layer)
    {
        // convert map to cv image
        cv::Mat img;
        grid_map::GridMapCvConverter::toImage<unsigned char, 1>(_map, _obstacle_layer, CV_8UC1,
                                                                img);

        int kernel_size = static_cast<int>(radius / _map.getResolution());
        cv::Mat kernel  = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1));

        cv::Mat inflated_img;
        cv::dilate(img, inflated_img, kernel);

        grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
            inflated_img, inflated_layer, _map, 0.0, 100.);

        // grid map is already checking if layer exists or not
        /*_map.add(inflated_layer);*/
        /**/
        /*for (grid_map::GridMapIterator it(_map); !it.isPastEnd(); ++it)*/
        /*{*/
        /*    grid_map::Position pos;*/
        /*    _map.getPosition(*it, pos);*/
        /**/
        /*    if (is_occupied(pos[0], pos[1], _obstacle_layer)) continue;*/
        /**/
        /*    for (grid_map::CircleIterator cit(_map, pos, radius); !cit.isPastEnd(); ++cit)*/
        /*    {*/
        /*        float &val = _map.at(inflated_layer, *cit);*/
        /*        val        = std::max(val, 1.0f);*/
        /*    }*/
        /*}*/
    }

    bool raycast(unsigned int sx, unsigned int sy, unsigned int ex, unsigned int ey, double &x,
                 double &y, const std::vector<unsigned char> &test_val,
                 const std::string &layer, unsigned int max_range = 1e6)
    {
        bool ray_hit = false;

        std::vector<double> s_vec = map_to_world(sx, sy);
        std::vector<double> e_vec = map_to_world(ex, ey);

        grid_map::Position start(s_vec[0], s_vec[1]);
        grid_map::Position end(e_vec[0], e_vec[1]);
        grid_map::Position dir = end - start;

        dir.normalize();

        // get indices in map
        grid_map::Index start_ind(sx, sy);
        grid_map::Index end_ind(ex, ey);

        // raycast several times with small purturbations in direction to ensure
        // thin obstacles are detected
        double min_dist  = 1e6;
        double theta_dir = atan2(dir(1), dir(0));

        // raycast
        Eigen::Vector2d ray_end = end;
        for (grid_map::LineIterator iterator(_map, start_ind, end_ind); !iterator.isPastEnd();
             ++iterator)
        {
            grid_map::Position pos;
            _map.getPosition(*iterator, pos);
            if (is_occupied(pos(0), pos(1), layer))
            {
                // if we hit an obstacle, check distance to start
                ray_hit = true;
                ray_end = pos;
                break;
            }
        }

        x = ray_end(0);
        y = ray_end(1);

        return ray_hit;
    }

   private:
};
typedef OccupancyGrid occupancy_grid_t;

/*inline occupancy_grid_t costmap_to_occgrid(const grid_map::GridMap &costmap,*/
/*                                           const std::string &obs_layer,*/
/*                                           const std::vector<unsigned char> &obs_values)*/
/*{*/
/*    occupancy_grid_t occ_grid(costmap, obs_layer, obs_values);*/
/**/
/*    return occ_grid;*/
/*}*/

}  // end namespace map_util
