#pragma once

#include <robust_fast_navigation/rfn_types.h>

#include <Eigen/Core>
// #include <boost/variant/variant.hpp>
// #include <grid_map_ros/GridMapRosConverter.hpp>
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_sdf/SignedDistance2d.hpp>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace map_util {

enum class Layer { kInflated, kObstacles };

// struct mimicing nav_msgs::OccupancyGrid
class OccupancyGrid {
private:
  enum class FH_direction { kRow, kCol };
  static constexpr double kINF = 1e9;

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
  std::optional<Eigen::MatrixXd> _sdf;

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

    if (!update_t(current[0], dx, x_min, x_max))
      return current;
    if (!update_t(current[1], dy, y_min, y_max))
      return current;

    int count = 0;
    t_max *= epsilon;

    while (is_occupied(current[0] + (t_max * epsilon) * dx,
                       current[1] + (t_max * epsilon) * dy, Layer::kInflated) &&
           count++ < 10)
      t_max *= epsilon;

    return {current[0] + (t_max)*dx, current[1] + (t_max)*dy};
  }

  double sdf_dist(double x, double y, Layer layer) {
    if (!_sdf) {
      _sdf = Eigen::MatrixXd(height, width);
      compute_sdf(_sdf.value(), layer);
    }

    // bilinear interpolation
    double mx = (x - origin_x) / resolution;
    double my = (y - origin_y) / resolution;

    int x0 = (int)std::floor(mx);
    int y0 = (int)std::floor(my);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    x0 = std::clamp(x0, 0, width - 1);
    x1 = std::clamp(x1, 0, width - 1);
    y0 = std::clamp(y0, 0, height - 1);
    y1 = std::clamp(y1, 0, height - 1);

    double tx = mx - std::floor(mx);
    double ty = my - std::floor(my);

    double v00 = _sdf.value()(y0, x0);
    double v10 = _sdf.value()(y0, x1);
    double v01 = _sdf.value()(y1, x0);
    double v11 = _sdf.value()(y1, x1);

    return (1 - tx) * (1 - ty) * v00 + tx * (1 - ty) * v10 +
           (1 - tx) * ty * v01 + tx * ty * v11;
  }

  void compute_sdf(Eigen::MatrixXd &sdf, Layer layer) {

    for (int row = 0; row < height; ++row) {
      auto is_occ_member = [&, this](unsigned int variable_ind) {
        bool is_occ;
        is_occ = is_occupied(variable_ind, row, layer);

        return is_occ ? 0 : kINF;
      };

      sdf.row(row) = FH_transform(FH_direction::kRow, is_occ_member);
    }

    // now do the same thing but in column direction with the resulting values..
    for (int col = 0; col < width; ++col) {
      auto row_dist_value = [&, this](unsigned int variable_ind) {
        return sdf(variable_ind, col);
      };

      sdf.col(col) = FH_transform(FH_direction::kCol, row_dist_value);
    }

    sdf.array() = sdf.array().sqrt() * resolution;
  }

  // Felzenszwalb and Huttelocher algorithm for distance transforms
  // https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf
  template <typename Func>
  Eigen::VectorXd FH_transform(FH_direction dir, Func f) {
    int dim = (dir == FH_direction::kRow) ? width : height;

    size_t k = 0;
    std::vector<int> lower_env_locs(dim);
    lower_env_locs[0] = 0;

    std::vector<double> boundary_locs(dim + 1);
    boundary_locs[0] = -kINF;
    boundary_locs[1] = kINF;

    Eigen::VectorXd one_dim_dists(dim);

    for (int ind = 1; ind < dim; ++ind) {
      // this wasn't in the paper but without it, sdf results
      // are not correct. f(ind) ended up polluting the
      // intersection
      if (f(ind) >= kINF) {
        continue;
      }

      double intersect;
      do {
        double numer =
            (f(ind) + ind * ind -
             (f(lower_env_locs[k]) + lower_env_locs[k] * lower_env_locs[k]));
        intersect = numer / (2 * ind - 2 * lower_env_locs[k]);

        if (intersect <= boundary_locs[k]) {
          // this should never happen, but just in case...
          if (k == 0) {
            break;
          }
          --k;
        }

      } while (intersect <= boundary_locs[k]);

      lower_env_locs[++k] = ind;
      boundary_locs[k] = intersect;
      boundary_locs[k + 1] = kINF;
    }

    // fill in values of distance transform
    k = 0;
    for (int ind = 0; ind < dim; ++ind) {
      while (boundary_locs[k + 1] < ind) {
        ++k;
      }

      one_dim_dists[ind] =
          (ind - lower_env_locs[k]) * (ind - lower_env_locs[k]) +
          f(lower_env_locs[k]);
    }

    return one_dim_dists;
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
    if (mx >= width || my >= height) {
      std::cout << mx << " " << my << std::endl;
      throw std::invalid_argument(
          "[cells_to_index] mx or my is greater than width or height");
    }

    return my * width + mx;
  }

  const unsigned char *get_data() const { return data.data(); }

  unsigned char get_cost(double x, double y, Layer layer) const {
    std::vector<unsigned int> cells = world_to_map(x, y);
    return get_cost(cells[0], cells[1], layer);
  }

  unsigned char get_cost(unsigned int mx, unsigned int my, Layer layer) const {
    return get_cost(cells_to_index(mx, my), layer);
  }

  double get_resolution() const { return resolution; }

  std::vector<double> get_origin() const { return {origin_x, origin_y}; }

  std::vector<int> get_size() const { return {width, height}; }

  unsigned char get_cost(unsigned int index, Layer layer) const {
    if (layer == Layer::kInflated)
      return data[index];
    else if (layer == Layer::kObstacles) {
      if (data[index] == occupied_values[0])
        return 0;
      return data[index];
    } else {
      std::string err = "[get_cost] layer must be kInflated or kObstacles";
      throw std::invalid_argument(err);
    }

    return data[index];
  }

  unsigned char get_cost(unsigned int index, Layer layer) {
    if (layer == Layer::kInflated)
      return data[index];
    else if (layer == Layer::kInflated) {
      if (data[index] == occupied_values[0])
        return 0;
      return data[index];
    } else {
      std::string err = "[get_cost] layer must be kInflated or kObstacles";
      throw std::invalid_argument(err);
    }

    return data[index];
  }

  bool is_occupied(double x, double y, Layer layer) const {
    std::vector<unsigned int> cells = world_to_map(x, y);
    return is_occupied(cells[0], cells[1], layer);
  }

  bool is_occupied(unsigned int mx, unsigned int my, Layer layer) const {
    return is_occupied(cells_to_index(mx, my), layer);
  }

  bool is_occupied(unsigned int index, Layer layer) const {
    unsigned char cost = get_cost(index, layer);
    return std::find(occupied_values.begin(), occupied_values.end(), cost) !=
           occupied_values.end();
  }

  bool raycast(unsigned int sx, unsigned int sy, unsigned int ex,
               unsigned int ey, double &x, double &y, Layer layer,
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
                 unsigned int max_range, unsigned int &term, Layer layer,
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

        if (is_occupied(i, j, Layer::kInflated))
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
};
typedef OccupancyGrid occupancy_grid_t;

} // end namespace map_util
