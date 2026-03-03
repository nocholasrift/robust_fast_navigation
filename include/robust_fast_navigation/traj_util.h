#ifndef ROBUST_FAST_NAVIGATION_TRAJ_UTIL_H
#define ROBUST_FAST_NAVIGATION_TRAJ_UTIL_H

#include <robust_fast_navigation/rfn_types.h>

namespace traj_utils {
inline double compute_arclen(const RFNTrajectory &traj, double t0, double tf) {
  // find arclength using trapezoid method
  double s = 0.0;
  double dt = (tf - t0) / 100.;

  Eigen::VectorXd prev_v = traj.getVel(t0);

  for (double t = t0 + dt; t < tf; t += dt) {
    Eigen::VectorXd curr_v = traj.getVel(t);

    // s += std::sqrt(dx * dx + dy * dy) * dt;
    s += .5 * (curr_v.norm() + prev_v.norm()) * dt;

    prev_v = curr_v;
  }

  return s;
}

inline double binary_search(const RFNTrajectory &traj, double dl, double start,
                            double end, double tolerance) {
  double t_left = start;
  double t_right = end;

  double prev_s = 0;
  double s = -1000;

  while (fabs(prev_s - s) > tolerance) {
    prev_s = s;

    double t_mid = (t_left + t_right) / 2;

    // always interested in total arc length up to t_mid
    s = compute_arclen(traj, 0, t_mid);

    // std::cout << "\ts at " << t_mid << " is " << s << std::endl;

    if (s < dl)
      t_left = t_mid;
    else
      t_right = t_mid;
  }

  return (t_left + t_right) / 2;
}

inline bool reparam_traj(const RFNTrajectory &traj, std::vector<double> &ss,
                         std::vector<double> &xs, std::vector<double> &ys) {

  if (traj.num_segments() == 0) {
    return false;
  }

  double traj_duration = traj.getDuration();

  double total_length = compute_arclen(traj, 0, traj_duration);

  double M = 20;
  double ds = total_length / M;

  ss.resize(M + 1);
  xs.resize(M + 1);
  ys.resize(M + 1);

  double previous_ti = 0;
  for (int i = 0; i <= M; ++i) {
    double s = i * ds;

    double ti;
    if (i == 0)
      ti = 0.0;
    else if (i == M)
      ti = traj_duration;
    else
      ti = binary_search(traj, s, previous_ti, traj_duration, 1e-3);

    Eigen::VectorXd pos = traj.getPos(ti);

    ss[i] = s;
    xs[i] = pos[0];
    ys[i] = pos[1];

    previous_ti = ti;
  }

  return true;
}

} // namespace traj_utils

#endif
