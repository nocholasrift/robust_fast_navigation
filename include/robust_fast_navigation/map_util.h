#pragma once

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <robust_fast_navigation/rfn_types.h>

#include <Eigen/Core>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_sdf/SignedDistance2d.hpp>
#include <vector>

namespace map_util
{
// struct mimicing nav_msgs::OccupancyGrid
class OccupancyGrid
{
   private:
    unsigned char *data;
    int width;
    int height;
    double resolution;
    double origin_x;
    double origin_y;

    grid_map::Matrix _sdf;

    std::vector<unsigned char> occupied_values;

   public:
    OccupancyGrid()
    {
        width      = 0;
        height     = 0;
        resolution = 0.0;
        origin_x   = 0.0;
        origin_y   = 0.0;
        data       = nullptr;
    }

    OccupancyGrid(int w, int h, double res, double ox, double oy, unsigned char *d,
                  std::vector<unsigned char> ov)
    {
        width           = w;
        height          = h;
        resolution      = res;
        origin_x        = ox;
        origin_y        = oy;
        data            = d;
        occupied_values = ov;
    }

    OccupancyGrid(const costmap_2d::Costmap2D &costmap)
    {
        width           = costmap.getSizeInCellsX();
        height          = costmap.getSizeInCellsY();
        resolution      = costmap.getResolution();
        origin_x        = costmap.getOriginX();
        origin_y        = costmap.getOriginY();
        occupied_values = {costmap_2d::LETHAL_OBSTACLE,
                           costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
        data            = costmap.getCharMap();

        Eigen::Matrix<bool, -1, -1> binary_map(height, width);
        for (unsigned int i = 0; i < height; i++)
        {
            for (unsigned int j = 0; j < width; j++)
            {
                unsigned char val = costmap.getCost(j, i);
                binary_map(i, j)  = val >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
            }
        }

        _sdf = grid_map::signed_distance_field::signedDistanceFromOccupancy(binary_map,
                                                                            resolution);
    }

    double get_signed_dist(double x, double y) const
    {
        std::vector<unsigned int> cells = world_to_map(x, y);
        unsigned int mx                 = cells[0];
        unsigned int my                 = cells[1];
        if (mx >= width || my >= height)
            throw std::invalid_argument(
                "[get_signed_distance] mx or my is greater than width or height");

        /*std::cout << "getting signed distance!" << std::endl;*/
        /*std::cout << "mx: " << mx << " my: " << my << std::endl;*/
        /*std::cout << "widht: " << _sdf.rows() << " height: " << _sdf.cols() << std::endl;*/
        return _sdf(my, mx);
    }

    std::vector<double> get_dist_grad(double x, double y) const
    {
        const double eps = resolution + 1e-2;

        double dx = (get_signed_dist(x + eps, y) - get_signed_dist(x - eps, y)) / (2.0 * eps);
        double dy = (get_signed_dist(x, y + eps) - get_signed_dist(x, y - eps)) / (2.0 * eps);

        Eigen::Vector2d grad(dx, dy);
        grad.normalize();

        return {grad[0], grad[1]};
    }

    // define these functions with vectors so we can pybind them more easily
    std::vector<unsigned int> world_to_map(double x, double y) const
    {
        if (x < origin_x || y < origin_y)
            throw std::invalid_argument("[world_to_map] x or y is less than origin");

        unsigned int mx = (int)((x - origin_x) / resolution);
        unsigned int my = (int)((y - origin_y) / resolution);

        if (mx >= width || my >= height)
            throw std::invalid_argument(
                "[world_to_map] mx or my is greater than width or height");

        return {mx, my};
    }

    std::vector<double> map_to_world(unsigned int mx, unsigned int my) const
    {
        if (mx > width || my > height)
            throw std::invalid_argument(
                "[map_to_world] mx or my is greater than width or height");

        double x = (mx + .5) * resolution + origin_x;
        double y = (my + .5) * resolution + origin_y;

        return {x, y};
    }

    std::vector<unsigned int> index_to_cells(unsigned int index) const
    {
        if (index > width * height)
            throw std::invalid_argument(
                "[index_to_cells] index is greater than width * height");

        unsigned int mx = index % width;
        unsigned int my = index / width;

        return {mx, my};
    }

    unsigned int cells_to_index(unsigned int mx, unsigned int my) const
    {
        /*std::cout << "[cells_to_index] mx: " << mx << " my: " << my << std::endl;*/
        if (mx > width || my > height)
            throw std::invalid_argument(
                "[cells_to_index] mx or my is greater than width or height");

        return my * width + mx;
    }

    unsigned char *get_data() const { return data; }

    unsigned char get_cost(double x, double y, const std::string &layer) const
    {
        std::vector<unsigned int> cells = world_to_map(x, y);
        return get_cost(cells[0], cells[1], layer);
    }

    unsigned char get_cost(unsigned int mx, unsigned int my, const std::string &layer) const
    {
        return get_cost(cells_to_index(mx, my), layer);
    }

    double get_resolution() const { return resolution; }

    std::vector<double> get_origin() const { return {origin_x, origin_y}; }

    std::vector<int> get_size() const { return {width, height}; }

    unsigned char get_cost(unsigned int index, const std::string &layer) const
    {
        if (layer == "inflated")
            return data[index];
        else if (layer == "obstacles")
        {
            if (data[index] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return 0;
            return data[index];
        }
        else
        {
            std::string err = "[get_cost] layer not found: " + layer;
            throw std::invalid_argument(err);
        }

        return data[index];
    }

    bool is_occupied(double x, double y, const std::string &layer) const
    {
        std::vector<unsigned int> cells = world_to_map(x, y);
        return is_occupied(cells[0], cells[1], layer);
    }

    bool is_occupied(unsigned int mx, unsigned int my, const std::string &layer) const
    {
        return is_occupied(cells_to_index(mx, my), layer);
    }

    bool is_occupied(unsigned int index, const std::string &layer) const
    {
        unsigned char cost = get_cost(index, layer);
        return std::find(occupied_values.begin(), occupied_values.end(), cost) !=
               occupied_values.end();
    }

    bool raycast(unsigned int sx, unsigned int sy, unsigned int ex, unsigned int ey, double &x,
                 double &y, const std::vector<unsigned char> &test_val,
                 const std::string &layer, unsigned int max_range = 1e6)
    {
        bool ray_hit        = false;
        unsigned int size_x = width;

        int dx = ex - sx;
        int dy = ey - sy;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = dx > 0 ? 1 : -1;
        int offset_dy = (dy > 0 ? 1 : -1) * size_x;

        unsigned int offset = sy * size_x + sx;

        double dist  = hypot(dx, dy);
        double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_range / dist);

        unsigned int term;
        if (abs_dx >= abs_dy)
        {
            int error_y = abs_dx / 2;
            ray_hit     = bresenham(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
                                    (unsigned int)(scale * abs_dx), term, layer, test_val);
        }
        else
        {
            int error_x = abs_dy / 2;
            ray_hit     = bresenham(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
                                    (unsigned int)(scale * abs_dy), term, layer, test_val);
        }

        // convert costmap index to world coordinates
        unsigned int mx, my;
        std::vector<unsigned int> cells = index_to_cells(term);
        mx                              = cells[0];
        my                              = cells[1];

        std::vector<double> world = map_to_world(mx, my);
        x                         = world[0];
        y                         = world[1];

        return ray_hit;
    }

    // following bresenham / raycast method from
    // https://docs.ros.org/en/api/costmap_2d/html/costmap__2d_8h_source.html
    bool bresenham(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                   int offset_b, unsigned int offset, unsigned int max_range,
                   unsigned int &term, const std::string layer,
                   const std::vector<unsigned char> &test_val)
    {
        bool ray_hit     = false;
        unsigned int end = std::min(max_range, abs_da);
        unsigned int mx, my;
        for (unsigned int i = 0; i < end; ++i)
        {
            offset += offset_a;
            error_b += abs_db;

            std::vector<unsigned int> cells = index_to_cells(offset);
            mx                              = cells[0];
            my                              = cells[1];

            unsigned char cost = get_cost(mx, my, layer);
            if (std::find(test_val.begin(), test_val.end(), cost) != test_val.end())
            {
                ray_hit = true;
                break;
            }

            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }

            cells = index_to_cells(offset);
            mx    = cells[0];
            my    = cells[1];

            cost = get_cost(mx, my, layer);
            if (std::find(test_val.begin(), test_val.end(), cost) != test_val.end())
            {
                ray_hit = true;
                break;
            }
        }

        term = offset;
        return ray_hit;
    }

    void push_trajectory(std::vector<rfn_state_t> &traj, double thresh_dist = .1,
                         int max_iters = 20)
    {
    }
};
typedef OccupancyGrid occupancy_grid_t;

}  // end namespace map_util
