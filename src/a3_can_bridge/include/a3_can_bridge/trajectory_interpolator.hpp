#pragma once

#include <cmath>
#include <cstddef>
#include <algorithm>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace a3_can_bridge
{

inline double DurationToSec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

/**
 * Sample a JointTrajectory at elapsed seconds from trajectory start.
 * Linear interpolation on positions/velocities/effort (same semantics as CloudEdge mock).
 * Returns false if trajectory has no points.
 */
inline bool SampleJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  double elapsed_s,
  std::vector<double> & positions_out,
  std::vector<double> & velocities_out,
  std::vector<double> & effort_out,
  bool & finished)
{
  finished = false;
  if (traj.points.empty()) {
    return false;
  }

  const auto & points = traj.points;
  const double t_end = DurationToSec(points.back().time_from_start);

  auto apply_point = [&](const trajectory_msgs::msg::JointTrajectoryPoint & pt) {
    positions_out = pt.positions;
    velocities_out = pt.velocities;
    effort_out = pt.effort;
  };

  if (elapsed_s >= t_end) {
    apply_point(points.back());
    finished = true;
    return true;
  }

  size_t idx = 0;
  while (idx + 1 < points.size() &&
    elapsed_s > DurationToSec(points[idx + 1].time_from_start))
  {
    ++idx;
  }

  if (idx + 1 >= points.size()) {
    apply_point(points.back());
    finished = true;
    return true;
  }

  const auto & p0 = points[idx];
  const auto & p1 = points[idx + 1];
  const double t0 = DurationToSec(p0.time_from_start);
  const double t1 = DurationToSec(p1.time_from_start);
  const double alpha = (t1 > t0) ? (elapsed_s - t0) / (t1 - t0) : 0.0;

  auto lerp_vec = [&](
    const std::vector<double> & a,
    const std::vector<double> & b,
    std::vector<double> & out)
  {
    const size_t n = std::max(a.size(), b.size());
    out.resize(n);
    for (size_t i = 0; i < n; ++i) {
      const double va = (i < a.size()) ? a[i] : 0.0;
      const double vb = (i < b.size()) ? b[i] : va;
      out[i] = va + alpha * (vb - va);
    }
  };

  lerp_vec(p0.positions, p1.positions, positions_out);
  lerp_vec(p0.velocities, p1.velocities, velocities_out);
  lerp_vec(p0.effort, p1.effort, effort_out);
  return true;
}

}  // namespace a3_can_bridge
