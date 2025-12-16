#pragma once

#include <robust_fast_navigation/rfn_types.h>

#include <Eigen/Core>
// #include <boost/variant/variant.hpp>
// #include <grid_map_ros/GridMapRosConverter.hpp>
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_sdf/SignedDistance2d.hpp>
#include <unordered_set>
#include <vector>

namespace map_util {
// struct mimicing nav_msgs::OccupancyGrid
class OccupancyGrid {
private:
  std::vector<unsigned char> data;
  int width;
  int height;
  double resolution;
  double origin_x;
  double origin_y;

  bool resized;
  bool use_sdf;

  int reset_counter;

  std::vector<unsigned char> occupied_values;
  std::vector<unsigned char> no_information_values;

  std::unordered_set<uint64_t> known_occupied_inds;

public:
  Eigen::MatrixXd _sdf;

  OccupancyGrid() {
    width = 0;
    height = 0;
    resolution = 0.0;
    origin_x = 0.0;
    origin_y = 0.0;

    resized = false;
    use_sdf = false;
    reset_counter = 0;
  }

  OccupancyGrid(int w, int h, double res, double ox, double oy,
                const std::vector<unsigned char> &d,
                const std::vector<unsigned char> &ov,
                const std::vector<unsigned char> &niv) {

    width = w;
    height = h;
    resolution = res;
    origin_x = ox;
    origin_y = oy;
    occupied_values = ov;
    no_information_values = niv;
    reset_counter = 0;

    data = d;

    update_occupied_obstacles();

    resized = false;
    this->use_sdf = false;
  }

  OccupancyGrid(int w, int h, double res, double ox, double oy,
                unsigned char *d, const std::vector<unsigned char> &ov,
                const std::vector<unsigned char> &niv) {
    width = w;
    height = h;
    resolution = res;
    origin_x = ox;
    origin_y = oy;
    occupied_values = ov;
    no_information_values = niv;
    reset_counter = 0;

    data = std::vector<unsigned char>(d, d + (w * h));

    update_occupied_obstacles();

    resized = false;
    this->use_sdf = false;
  }

  void update(int w, int h, double res, double ox, double oy,
              const std::vector<unsigned char> &d,
              const std::vector<unsigned char> &ov,
              const std::vector<unsigned char> &niv) {
    if (w != width || h != height || origin_x != ox || origin_y != oy ||
        reset_counter++ > 10) {
      std::cout
          << "[OccupancyGrid] Costmap metadata changed, updating occupancies"
          << std::endl;
      // reset cache since map has changed geometry
      known_occupied_inds.clear();
      reset_counter = 0;
    }

    width = w;
    height = h;
    resolution = res;
    origin_x = ox;
    origin_y = oy;
    occupied_values = ov;
    no_information_values = niv;
    data = d;

    update_occupied_obstacles();
  }

  void update(int w, int h, double res, double ox, double oy, unsigned char *d,
              const std::vector<unsigned char> &ov,
              const std::vector<unsigned char> &niv) {
    if (w != width || h != height || origin_x != ox || origin_y != oy ||
        reset_counter++ > 10) {
      std::cout
          << "[OccupancyGrid] Costmap metadata changed, updating occupancies"
          << std::endl;
      // reset cache since map has changed geometry
      known_occupied_inds.clear();
      reset_counter = 0;
    }

    width = w;
    height = h;
    resolution = res;
    origin_x = ox;
    origin_y = oy;
    occupied_values = ov;
    no_information_values = niv;
    data = std::vector<unsigned char>(d, d + (w * h));

    update_occupied_obstacles();
  }

  // std::vector<double> clamp_point_to_bounds(const std::vector<double>& goal)
  // {
  //     std::vector<double> clamped = goal;
  //
  //     double wx_max = origin_x + width * resolution;
  //     double wy_max = origin_y + height * resolution;
  //
  //     clamped[0] = std::min(std::max(clamped[0], origin_x), wx_max);
  //     clamped[1] = std::min(std::max(clamped[1], origin_y), wy_max);
  //
  //     return clamped;
  // }
  std::vector<double> clamp_point_to_bounds(const std::vector<double> &current,
                                            const std::vector<double> &goal) {
    double epsilon = .95;
    double x_min = origin_x;
    double y_min = origin_y;
    double x_max = x_min + (width)*resolution;
    double y_max = y_min + (height)*resolution;

    double dx = goal[0] - current[0];
    double dy = goal[1] - current[1];

    double t_min = 0.0, t_max = 1.0;

    auto update_t = [&](double p, double dp, double min_b,
                        double max_b) -> bool {
      if (std::abs(dp) < 1e-8)
        return true;
      double t0 = (min_b - p) / dp;
      double t1 = (max_b - p) / dp;
      if (t0 > t1)
        std::swap(t0, t1);
      t_min = std::max(t_min, t0);
      t_max = std::min(t_max, t1);
      return t_min <= t_max;
    };

    std::cout << "updating\n";
    if (!update_t(current[0], dx, x_min, x_max))
      return current;
    if (!update_t(current[1], dy, y_min, y_max))
      return current;
    std::cout << "done\n";

    int count = 0;
    t_max *= epsilon;

    std::cout << "occupancy testing\n";
    while (is_occupied(current[0] + (t_max * epsilon) * dx,
                       current[1] + (t_max * epsilon) * dy, "inflated") &&
           count++ < 10)
      t_max *= epsilon;

    return {current[0] + (t_max)*dx, current[1] + (t_max)*dy};
  }

  // define these functions with vectors so we can pybind them more easily
  std::vector<unsigned int> world_to_map(double x, double y) const {
    if (x < origin_x || y < origin_y) {
      std::cout << "[world_to_map] x: " << x << " y: " << y
                << " origin_x: " << origin_x << " origin_y: " << origin_y
                << std::endl;
      throw std::invalid_argument("[world_to_map] x or y is less than origin");
    }

    unsigned int mx = (int)((x - origin_x) / resolution);
    unsigned int my = (int)((y - origin_y) / resolution);

    if (mx >= width || my >= height) {
      std::cout << "x: " << x << " y: " << y << " mx: " << mx << " my: " << my
                << " origin_x: " << origin_x << " origin_y: " << origin_y
                << " resolution: " << resolution << std::endl;
      std::cout << "[world_to_map] mx: " << mx << " my: " << my
                << " width: " << width << " height: " << height << std::endl;
      throw std::invalid_argument(
          "[world_to_map] mx or my is greater than width or height");
    }

    return {mx, my};
  }

  std::vector<double> map_to_world(unsigned int mx, unsigned int my) const {
    if (mx > width || my > height)
      throw std::invalid_argument(
          "[map_to_world] mx or my is greater than width or height");

    double x = (mx + .5) * resolution + origin_x;
    double y = (my + .5) * resolution + origin_y;

    return {x, y};
  }

  std::vector<unsigned int> index_to_cells(unsigned int index) const {
    if (index > width * height)
      throw std::invalid_argument(
          "[index_to_cells] index is greater than width * height");

    unsigned int mx = index % width;
    unsigned int my = index / width;

    return {mx, my};
  }

  unsigned int cells_to_index(unsigned int mx, unsigned int my) const {
    /*std::cout << "[cells_to_index] mx: " << mx << " my: " << my <<
     * std::endl;*/
    if (mx > width || my > height) {
      std::cout << mx << " " << my << std::endl;
      throw std::invalid_argument(
          "[cells_to_index] mx or my is greater than width or height");
    }

    return my * width + mx;
  }

  const unsigned char *get_data() const { return data.data(); }

  unsigned char get_cost(double x, double y, const std::string &layer) const {
    std::vector<unsigned int> cells = world_to_map(x, y);
    return get_cost(cells[0], cells[1], layer);
  }

  unsigned char get_cost(unsigned int mx, unsigned int my,
                         const std::string &layer) const {
    return get_cost(cells_to_index(mx, my), layer);
  }

  double get_resolution() const { return resolution; }

  std::vector<double> get_origin() const { return {origin_x, origin_y}; }

  std::vector<int> get_size() const { return {width, height}; }

  unsigned char get_cost(unsigned int index, const std::string &layer) const {
    if (layer == "inflated")
      return data[index];
    else if (layer == "obstacles") {
      if (data[index] == occupied_values[0])
        return 0;
      return data[index];
    } else {
      std::string err = "[get_cost] layer not found: " + layer;
      throw std::invalid_argument(err);
    }

    return data[index];
  }

  unsigned char get_cost(unsigned int index, const std::string &layer) {
    if (layer == "inflated")
      return data[index];
    else if (layer == "obstacles") {
      if (data[index] == occupied_values[0])
        return 0;
      return data[index];
    } else {
      std::string err = "[get_cost] layer not found: " + layer;
      throw std::invalid_argument(err);
    }

    return data[index];
  }

  bool is_occupied(double x, double y, const std::string &layer) const {
    std::vector<unsigned int> cells = world_to_map(x, y);
    return is_occupied(cells[0], cells[1], layer);
  }

  bool is_occupied(unsigned int mx, unsigned int my,
                   const std::string &layer) const {
    return is_occupied(cells_to_index(mx, my), layer);
  }

  bool is_occupied(unsigned int index, const std::string &layer) const {
    unsigned char cost = get_cost(index, layer);
    return std::find(occupied_values.begin(), occupied_values.end(), cost) !=
           occupied_values.end();
  }

  bool raycast(unsigned int sx, unsigned int sy, unsigned int ex,
               unsigned int ey, double &x, double &y, const std::string &layer,
               std::vector<unsigned char> *test_val = nullptr,
               unsigned int max_range = 1e6) {

    if (!test_val) {
      test_val = &occupied_values;
    }

    bool ray_hit = false;
    unsigned int size_x = width;

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
    if (abs_dx >= abs_dy) {
      int error_y = abs_dx / 2;
      ray_hit =
          bresenham(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
                    (unsigned int)(scale * abs_dx), term, layer, *test_val);
    } else {
      int error_x = abs_dy / 2;
      ray_hit =
          bresenham(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
                    (unsigned int)(scale * abs_dy), term, layer, *test_val);
    }

    // convert costmap index to world coordinates
    unsigned int mx, my;
    std::vector<unsigned int> cells = index_to_cells(term);
    mx = cells[0];
    my = cells[1];

    std::vector<double> world = map_to_world(mx, my);
    x = world[0];
    y = world[1];

    return ray_hit;
  }

  // following bresenham / raycast method from
  // https://docs.ros.org/en/api/costmap_2d/html/costmap__2d_8h_source.html
  bool bresenham(unsigned int abs_da, unsigned int abs_db, int error_b,
                 int offset_a, int offset_b, unsigned int offset,
                 unsigned int max_range, unsigned int &term,
                 const std::string layer,
                 const std::vector<unsigned char> &test_val) {
    bool ray_hit = false;
    unsigned int end = std::min(max_range, abs_da);
    unsigned int mx, my;
    for (unsigned int i = 0; i < end; ++i) {
      offset += offset_a;
      error_b += abs_db;

      std::vector<unsigned int> cells = index_to_cells(offset);
      mx = cells[0];
      my = cells[1];

      unsigned char cost = get_cost(mx, my, layer);
      if (std::find(test_val.begin(), test_val.end(), cost) != test_val.end()) {
        ray_hit = true;
        break;
      }

      if ((unsigned int)error_b >= abs_da) {
        offset += offset_b;
        error_b -= abs_da;
      }

      cells = index_to_cells(offset);
      mx = cells[0];
      my = cells[1];

      cost = get_cost(mx, my, layer);
      if (std::find(test_val.begin(), test_val.end(), cost) != test_val.end()) {
        ray_hit = true;
        break;
      }
    }

    term = offset;
    return ray_hit;
  }

  std::vector<Eigen::VectorXd> get_occupied(uint8_t dims) const {
    if (dims != 2 && dims != 3) {
      std::cout << "[getOccupied] dims must be 2 or 3" << std::endl;
      return {};
    }

    std::vector<Eigen::VectorXd> paddedObs;
    paddedObs.reserve(known_occupied_inds.size());

    // iterate through known_occupied_inds
    for (const auto &idx : known_occupied_inds) {
      unsigned int mx, my;
      std::vector<unsigned int> cells = index_to_cells(idx);
      mx = cells[0];
      my = cells[1];
      // convert to world coordinates
      std::vector<double> coords = map_to_world(mx, my);
      double x = coords[0];
      double y = coords[1];

      Eigen::VectorXd &point = paddedObs.emplace_back(dims);
      point(0) = x;
      point(1) = y;
      if (dims == 3)
        point(2) = 0.0;
    }

    return paddedObs;
  }

  void update_occupied_obstacles() {
    for (unsigned int j = 0; j < height; ++j) {
      for (unsigned int i = 0; i < width; ++i) {
        unsigned int idx = cells_to_index(i, j);
        if (known_occupied_inds.find(idx) != known_occupied_inds.end()) {
          continue;
        }

        if (is_occupied(i, j, "inflated"))
          known_occupied_inds.insert(idx);
      }
    }

    std::cout << "[OccupancyGrid] Found " << known_occupied_inds.size()
              << " occupied cells " << std::endl;
  }

  std::vector<unsigned char> get_occupied_values() const {
    return occupied_values;
  }

  std::vector<unsigned char> get_no_info_values() const {
    return no_information_values;
  }

  // void push_trajectory(std::vector<rfn_state_t> &traj, double thresh_dist =
  // .1,
  //                      int max_iters = 100) {
  //   if (!use_sdf)
  //     throw std::invalid_argument(
  //         "[get_signed_distance] SDF not generated, use_sdf is false");
  //
  //   double step_size = resolution / 2.0;
  //   for (rfn_state_t &x : traj) {
  //     // perform gradient descent on point if it is too close to obstacles
  //     double dist;
  //     try {
  //       dist = get_signed_dist(x.pos(0), x.pos(1));
  //     } catch (...) {
  //       return;
  //     }
  //     Eigen::Vector2d p = x.pos.head(2);
  //     if (dist < thresh_dist) {
  //       for (int i = 0; i < max_iters && dist < thresh_dist; ++i) {
  //         // get normalized gradient vector to obstacle
  //         std::vector<double> d = get_dist_grad(p(0), p(1));
  //         Eigen::Vector2d grad = {d[0], d[1]};
  //         try {
  //           Eigen::Vector2d tmp = p + step_size * grad;
  //           dist = get_signed_dist(tmp[0], tmp[1]);
  //         } catch (...) {
  //           x.pos.head(2) = p;
  //           return;
  //         }
  //
  //         p = p + step_size * grad;
  //       }
  //
  //       // update trajectory
  //       x.pos.head(2) = p;
  //     }
  //   }
  // }
};
typedef OccupancyGrid occupancy_grid_t;

} // end namespace map_util
