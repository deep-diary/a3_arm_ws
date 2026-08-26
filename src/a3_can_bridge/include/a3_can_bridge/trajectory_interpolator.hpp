#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
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

/** JTC-aligned: auto | linear | cubic | quintic */
enum class TrajInterpMethod
{
  Auto,
  Linear,
  Cubic,
  Quintic
};

inline TrajInterpMethod ParseTrajInterpMethod(const std::string & s)
{
  if (s == "linear") {
    return TrajInterpMethod::Linear;
  }
  if (s == "cubic") {
    return TrajInterpMethod::Cubic;
  }
  if (s == "quintic") {
    return TrajInterpMethod::Quintic;
  }
  return TrajInterpMethod::Auto;
}

inline bool SegmentHasVel(
  const trajectory_msgs::msg::JointTrajectoryPoint & p0,
  const trajectory_msgs::msg::JointTrajectoryPoint & p1)
{
  return !p0.velocities.empty() && !p1.velocities.empty();
}

inline bool SegmentHasAcc(
  const trajectory_msgs::msg::JointTrajectoryPoint & p0,
  const trajectory_msgs::msg::JointTrajectoryPoint & p1)
{
  return !p0.accelerations.empty() && !p1.accelerations.empty();
}

inline TrajInterpMethod ResolveMethod(
  TrajInterpMethod requested,
  const trajectory_msgs::msg::JointTrajectoryPoint & p0,
  const trajectory_msgs::msg::JointTrajectoryPoint & p1)
{
  if (requested == TrajInterpMethod::Linear) {
    return TrajInterpMethod::Linear;
  }
  if (requested == TrajInterpMethod::Cubic) {
    return SegmentHasVel(p0, p1) ? TrajInterpMethod::Cubic : TrajInterpMethod::Linear;
  }
  if (requested == TrajInterpMethod::Quintic) {
    if (SegmentHasVel(p0, p1) && SegmentHasAcc(p0, p1)) {
      return TrajInterpMethod::Quintic;
    }
    if (SegmentHasVel(p0, p1)) {
      return TrajInterpMethod::Cubic;
    }
    return TrajInterpMethod::Linear;
  }
  // Auto (JTC): pos+vel+acc → quintic; pos+vel → cubic; else linear
  if (SegmentHasVel(p0, p1) && SegmentHasAcc(p0, p1)) {
    return TrajInterpMethod::Quintic;
  }
  if (SegmentHasVel(p0, p1)) {
    return TrajInterpMethod::Cubic;
  }
  return TrajInterpMethod::Linear;
}

inline double AtOr(const std::vector<double> & v, size_t i, double fallback = 0.0)
{
  return i < v.size() ? v[i] : fallback;
}

/** Sample one DoF with JTC-style spline on u∈[0,1], dt = t1-t0. */
inline void SampleDof(
  TrajInterpMethod method, double u, double dt,
  double p0, double p1, double v0, double v1, double a0, double a1,
  double & p_out, double & v_out)
{
  u = std::clamp(u, 0.0, 1.0);
  if (method == TrajInterpMethod::Linear || dt <= 1e-12) {
    p_out = p0 + u * (p1 - p0);
    v_out = (dt > 1e-12) ? (p1 - p0) / dt : 0.0;
    return;
  }

  const double u2 = u * u;
  const double u3 = u2 * u;

  if (method == TrajInterpMethod::Cubic) {
    // Hermite: p(u) = h00 p0 + h10 (dt v0) + h01 p1 + h11 (dt v1)
    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 = u3 - u2;
    p_out = h00 * p0 + h10 * (dt * v0) + h01 * p1 + h11 * (dt * v1);
    // dp/du
    const double dh00 = 6.0 * u2 - 6.0 * u;
    const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
    const double dh01 = -6.0 * u2 + 6.0 * u;
    const double dh11 = 3.0 * u2 - 2.0 * u;
    const double dp_du = dh00 * p0 + dh10 * (dt * v0) + dh01 * p1 + dh11 * (dt * v1);
    v_out = dp_du / dt;
    return;
  }

  // Quintic Hermite (pos, vel, acc at endpoints), parameter u in [0,1]
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;
  const double h1 = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5;
  const double h2 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5;
  const double h3 = 0.5 * u3 - u4 + 0.5 * u5;
  const double h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5;
  const double h5 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  p_out = h0 * p0 + h1 * (dt * v0) + h2 * (dt * dt * a0) +
    h3 * (dt * dt * a1) + h4 * (dt * v1) + h5 * p1;

  const double dh0 = -30.0 * u2 + 60.0 * u3 - 30.0 * u4;
  const double dh1 = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4;
  const double dh2 = u - 4.5 * u2 + 6.0 * u3 - 2.5 * u4;
  const double dh3 = 1.5 * u2 - 4.0 * u3 + 2.5 * u4;
  const double dh4 = -12.0 * u2 + 28.0 * u3 - 15.0 * u4;
  const double dh5 = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
  const double dp_du = dh0 * p0 + dh1 * (dt * v0) + dh2 * (dt * dt * a0) +
    dh3 * (dt * dt * a1) + dh4 * (dt * v1) + dh5 * p1;
  v_out = dp_du / dt;
}

/**
 * Sample a JointTrajectory at elapsed seconds from trajectory start.
 * Positions use JTC-compatible spline; effort always linear.
 * Returns false if trajectory has no points.
 */
inline bool SampleJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory & traj,
  double elapsed_s,
  std::vector<double> & positions_out,
  std::vector<double> & velocities_out,
  std::vector<double> & effort_out,
  bool & finished,
  TrajInterpMethod method = TrajInterpMethod::Auto)
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
  const double dt = t1 - t0;
  const double u = (dt > 1e-12) ? (elapsed_s - t0) / dt : 0.0;
  const TrajInterpMethod use = ResolveMethod(method, p0, p1);

  const size_t n = std::max(p0.positions.size(), p1.positions.size());
  positions_out.resize(n);
  velocities_out.resize(n);
  for (size_t i = 0; i < n; ++i) {
    SampleDof(
      use, u, dt,
      AtOr(p0.positions, i), AtOr(p1.positions, i),
      AtOr(p0.velocities, i), AtOr(p1.velocities, i),
      AtOr(p0.accelerations, i), AtOr(p1.accelerations, i),
      positions_out[i], velocities_out[i]);
  }

  // Effort: always linear (JTC)
  const size_t ne = std::max(p0.effort.size(), p1.effort.size());
  effort_out.resize(ne);
  for (size_t i = 0; i < ne; ++i) {
    const double e0 = AtOr(p0.effort, i);
    const double e1 = AtOr(p1.effort, i, e0);
    effort_out[i] = e0 + u * (e1 - e0);
  }
  return true;
}

}  // namespace a3_can_bridge
