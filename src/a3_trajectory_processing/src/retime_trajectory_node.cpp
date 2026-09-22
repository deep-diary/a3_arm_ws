// F68: geometric-preserving re-timing service.
// Dense recorded paths (positions only) are re-timed with Ruckig (jerk-limited,
// default) or TOTG. Waypoint timing is seeded from the recorded time_from_start
// and finite-difference velocities/accelerations are derived, so the recorded
// velocity need not be stored.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <a3_msgs/srv/retime_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
constexpr const char* kServiceName = "/a3/arm/retime_trajectory";

double point_time(const trajectory_msgs::msg::JointTrajectoryPoint& p)
{
  return static_cast<double>(p.time_from_start.sec) +
         static_cast<double>(p.time_from_start.nanosec) * 1e-9;
}
}  // namespace

class RetimeTrajectoryNode : public rclcpp::Node
{
public:
  RetimeTrajectoryNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("a3_trajectory_processing", options)
  {
    // NodeOptions.automatically_declare_parameters_from_overrides 会预声明 launch
    // 下发的同名参数，这里直接 declare 会抛 ParameterAlreadyDeclaredException
    group_name_ = has_parameter("group_name")
                      ? get_parameter("group_name").as_string()
                      : declare_parameter<std::string>("group_name", "arm_with_gripper");
    default_jerk_scale_ = has_parameter("default_jerk_scale")
                              ? get_parameter("default_jerk_scale").as_double()
                              : declare_parameter<double>("default_jerk_scale", 5.0);
    seed_jerk_margin_ = has_parameter("seed_jerk_margin")
                            ? get_parameter("seed_jerk_margin").as_double()
                            : declare_parameter<double>("seed_jerk_margin", 1.5);
  }

  // 必须在 make_shared 之后调用（构造函数里 shared_from_this() 会 bad_weak_ptr）
  void init()
  {
    robot_model_loader::RobotModelLoader::Options loader_options;
    loader_options.load_kinematics_solvers_ = false;
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), loader_options);
    robot_model_ = loader_->getModel();
    if (!robot_model_)
    {
      throw std::runtime_error("RetimeTrajectoryNode: failed to load robot model");
    }
    joint_model_group_ = robot_model_->getJointModelGroup(group_name_);
    if (!joint_model_group_)
    {
      throw std::runtime_error("RetimeTrajectoryNode: joint model group '" + group_name_ + "' not found");
    }

    service_ = create_service<a3_msgs::srv::RetimeTrajectory>(
        kServiceName,
        [this](const std::shared_ptr<a3_msgs::srv::RetimeTrajectory::Request> request,
               std::shared_ptr<a3_msgs::srv::RetimeTrajectory::Response> response) {
          handle_request(request, response);
        });

    RCLCPP_INFO(get_logger(), "serving %s (group=%s, %zu joints)", kServiceName, group_name_.c_str(),
                joint_model_group_->getActiveJointModels().size());
  }

private:
  using InputTraj = trajectory_msgs::msg::JointTrajectory;

  bool validate_input(const InputTraj& input, std::string& error)
  {
    if (input.points.size() < 2)
    {
      error = "need at least 2 trajectory points";
      return false;
    }
    const auto& group_joint_names = joint_model_group_->getActiveJointModelNames();
    std::unordered_map<std::string, size_t> input_index;
    for (size_t i = 0; i < input.joint_names.size(); ++i)
    {
      input_index[input.joint_names[i]] = i;
    }
    for (const auto& name : group_joint_names)
    {
      if (input_index.find(name) == input_index.end())
      {
        error = "input trajectory missing group joint: " + name;
        return false;
      }
      if (input.points.front().positions.size() <= input_index[name])
      {
        error = "first point has no position for joint " + name;
        return false;
      }
    }
    return true;
  }

  // 用录制时刻 + 差分速度/加速度播种：Ruckig 的 Humble 实现逐点做一次 update，
  // 无定时（dt≈0）或每点速度恒 0 的稠密路径必然 Working（error 0）。
  // have_times_out 报告输入是否带严格递增的录制时刻（决定是否放宽 jerk 上限）。
  robot_trajectory::RobotTrajectory build_seeded_trajectory(const InputTraj& input, double fallback_dt,
                                                           bool& have_times_out)
  {
    const auto& joints = joint_model_group_->getActiveJointModels();
    const size_t n = input.points.size();
    const size_t m = joints.size();

    std::vector<double> t(n);
    bool have_times = true;
    for (size_t i = 0; i < n; ++i)
    {
      t[i] = point_time(input.points[i]);
      if (i > 0 && !(t[i] > t[i - 1]))
      {
        have_times = false;
      }
    }
    have_times_out = have_times;
    std::vector<double> dt(n, fallback_dt);
    if (have_times)
    {
      for (size_t i = 1; i < n; ++i)
      {
        dt[i] = std::max(1e-4, t[i] - t[i - 1]);
      }
    }

    std::vector<std::vector<double>> q(n, std::vector<double>(m));
    std::unordered_map<std::string, size_t> idx;
    for (size_t k = 0; k < input.joint_names.size(); ++k)
    {
      idx[input.joint_names[k]] = k;
    }
    for (size_t i = 0; i < n; ++i)
    {
      for (size_t j = 0; j < m; ++j)
      {
        q[i][j] = input.points[i].positions[idx[joints[j]->getName()]];
      }
    }

    std::vector<std::vector<double>> v(n, std::vector<double>(m, 0.0));
    std::vector<std::vector<double>> a(n, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < n; ++i)
    {
      for (size_t j = 0; j < m; ++j)
      {
        if (i == 0)
        {
          v[i][j] = (q[1][j] - q[0][j]) / dt[1];
        }
        else if (i + 1 < n)
        {
          v[i][j] = (q[i + 1][j] - q[i - 1][j]) / (dt[i] + dt[i + 1]);
        }
        else
        {
          v[i][j] = (q[n - 1][j] - q[n - 2][j]) / dt[n - 1];
        }
      }
    }
    for (size_t i = 0; i < n; ++i)
    {
      for (size_t j = 0; j < m; ++j)
      {
        if (i == 0)
        {
          a[i][j] = (v[1][j] - v[0][j]) / dt[1];
        }
        else
        {
          a[i][j] = (v[i][j] - v[i - 1][j]) / dt[i];
        }
      }
    }

    robot_trajectory::RobotTrajectory traj(robot_model_, joint_model_group_);
    for (size_t i = 0; i < n; ++i)
    {
      moveit::core::RobotState state(robot_model_);
      state.setToDefaultValues();
      for (size_t j = 0; j < m; ++j)
      {
        state.setJointPositions(joints[j], &q[i][j]);
        state.setJointVelocities(joints[j], &v[i][j]);
        state.setVariableAcceleration(joints[j]->getFirstVariableIndex(), a[i][j]);
      }
      state.update();
      traj.addSuffixWayPoint(state, dt[i]);
    }
    return traj;
  }

  robot_trajectory::RobotTrajectory build_unseeded_trajectory(const InputTraj& input)
  {
    const auto& joints = joint_model_group_->getActiveJointModels();
    std::unordered_map<std::string, size_t> idx;
    for (size_t k = 0; k < input.joint_names.size(); ++k)
    {
      idx[input.joint_names[k]] = k;
    }
    robot_trajectory::RobotTrajectory traj(robot_model_, joint_model_group_);
    for (const auto& point : input.points)
    {
      moveit::core::RobotState state(robot_model_);
      state.setToDefaultValues();
      for (const auto& joint : joints)
      {
        const double value = point.positions[idx[joint->getName()]];
        state.setJointPositions(joint, &value);
      }
      state.zeroVelocities();
      state.zeroAccelerations();
      traj.addSuffixWayPoint(state, 0.001);
    }
    return traj;
  }

  void handle_request(const std::shared_ptr<a3_msgs::srv::RetimeTrajectory::Request>& request,
                      std::shared_ptr<a3_msgs::srv::RetimeTrajectory::Response>& response)
  {
    const auto& input = request->trajectory;
    std::string validation_error;
    if (!validate_input(input, validation_error))
    {
      response->success = false;
      response->message = validation_error;
      return;
    }

    const auto& group_joint_names = joint_model_group_->getActiveJointModelNames();

    double v_scaling = (request->velocity_scaling > 0.0 && request->velocity_scaling <= 1.0)
                           ? request->velocity_scaling
                           : 1.0;
    double a_scaling = (request->acceleration_scaling > 0.0 && request->acceleration_scaling <= 1.0)
                           ? request->acceleration_scaling
                           : 1.0;
    double sample_dt = request->sample_dt > 0.0 ? request->sample_dt : 0.01;

    std::unordered_map<std::string, double> base_velocity_limits;
    std::unordered_map<std::string, double> base_acceleration_limits;
    std::unordered_map<std::string, double> base_jerk_limits;
    if (!build_limits(group_joint_names, v_scaling, a_scaling, base_velocity_limits,
                      base_acceleration_limits, base_jerk_limits))
    {
      response->success = false;
      response->message = "missing joint limit parameters (set velocity_limits/acceleration_limits at launch)";
      return;
    }

    std::string backend = request->backend;
    if (backend.empty())
    {
      backend = "ruckig";
    }
    std::transform(backend.begin(), backend.end(), backend.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    bool ok = false;
    std::string used_backend;
    if (backend == "ruckig")
    {
      bool have_times = false;
      auto seeded = build_seeded_trajectory(input, sample_dt, have_times);
      // 录制时刻存在时，配置的 jerk 上限若比录制路径的差分 jerk 更紧，Ruckig 会
      // 全局拉长时长（实测 4s -> 29.6s），与"匹配录制时长"目标冲突。逐关节取
      // max(配置 jerk, 录制峰值 jerk)；需要延长时仍受更大的 jerk 约束。
      auto effective_jerk = base_jerk_limits;
      if (have_times)
      {
        relax_jerk_for_seed(seeded, effective_jerk);
      }
      ok = trajectory_processing::RuckigSmoothing::applySmoothing(seeded, base_velocity_limits,
                                                                  base_acceleration_limits, effective_jerk);
      if (ok)
      {
        response->trajectory = to_msg(seeded, group_joint_names);
      }
      used_backend = "ruckig";
      if (!ok)
      {
        RCLCPP_WARN(get_logger(), "ruckig smoothing failed, falling back to TOTG");
      }
    }

    double final_duration = 0.0;
    if (!ok)
    {
      used_backend = "totg";
      double vf = v_scaling;
      double af = a_scaling;
      const double target = request->target_duration;
      const bool match = target > 0.0;
      // 短稠密路径 TOTG 时长近似 ∝ 1/sqrt(a)：单轮线性缩放无法命中目标时长，
      // 实测时长 → 按 d^-2 修正 a 因子，迭代至多 5 轮。
      for (int round = 0; round < 5; ++round)
      {
        auto traj = build_unseeded_trajectory(input);
        auto vmap = scale_map(base_velocity_limits, vf / v_scaling);
        auto amap = scale_map(base_acceleration_limits, af / a_scaling);
        const trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.05, sample_dt, 0.001);
        if (!totg.computeTimeStamps(traj, vmap, amap))
        {
          if (round == 0)
          {
            response->success = false;
            response->message = backend + " re-timing failed";
            return;
          }
          break;
        }
        final_duration = traj.getWayPointDurationFromStart(traj.getWayPointCount() - 1);
        if (!match || std::abs(final_duration - target) <= target * 0.05)
        {
          response->trajectory = to_msg(traj, group_joint_names);
          ok = true;
          break;
        }
        const double ratio = final_duration / target;
        af = std::clamp(af * ratio * ratio, 1e-3, 1.0);
        vf = std::clamp(vf * ratio, 1e-3, 1.0);
        if (round == 4)
        {
          response->trajectory = to_msg(traj, group_joint_names);
          ok = true;
        }
      }
    }

    if (!ok)
    {
      response->success = false;
      response->message = backend + " re-timing failed";
      return;
    }

    response->success = true;
    std::ostringstream oss;
    oss << used_backend << ": " << input.points.size() << " pts -> "
        << response->trajectory.points.size() << " pts, "
        << std::fixed << std::setprecision(2);
    if (used_backend == "ruckig")
    {
      const double d = point_time(response->trajectory.points.back());
      oss << d;
      final_duration = d;
    }
    else
    {
      oss << final_duration;
    }
    oss << "s";
    response->message = oss.str();
  }

  // Ruckig 每点只做一次固定 dt 的 update，必须在一个常 jerk 段内同时精确到达
  // 末端的 (p, v, a)。差分播种的三态彼此只近似一致，各自所需的单步 jerk 为：
  //   j_a = da/dt
  //   j_v = 2(dv - a0 dt)/dt^2
  //   j_p = 6(dp - v0 dt - .5 a0 dt^2)/dt^3
  // 取三者峰值，jerk 上限不得比它更紧，否则触发全局时长拉伸。
  void relax_jerk_for_seed(const robot_trajectory::RobotTrajectory& seeded,
                           std::unordered_map<std::string, double>& jerk_limits)
  {
    const auto* group = seeded.getGroup();
    const auto& idx = group->getVariableIndexList();
    const auto& vars = group->getVariableNames();
    const size_t n = seeded.getWayPointCount();
    for (size_t v = 0; v < vars.size(); ++v)
    {
      double peak = 0.0;
      for (size_t i = 1; i < n; ++i)
      {
        const auto& s0 = seeded.getWayPoint(i - 1);
        const auto& s1 = seeded.getWayPoint(i);
        const double dt = seeded.getWayPointDurationFromPrevious(i);
        if (dt <= 0.0)
        {
          continue;
        }
        const double p0 = s0.getVariablePosition(idx.at(v));
        const double p1 = s1.getVariablePosition(idx.at(v));
        const double v0 = s0.getVariableVelocity(idx.at(v));
        const double v1 = s1.getVariableVelocity(idx.at(v));
        const double a0 = s0.getVariableAcceleration(idx.at(v));
        const double a1 = s1.getVariableAcceleration(idx.at(v));
        const double j_a = (a1 - a0) / dt;
        const double j_v = 2.0 * ((v1 - v0) - a0 * dt) / (dt * dt);
        const double j_p = 6.0 * ((p1 - p0) - v0 * dt - 0.5 * a0 * dt * dt) / (dt * dt * dt);
        peak = std::max(peak, std::max(std::abs(j_a), std::max(std::abs(j_v), std::abs(j_p))));
      }
      auto it = jerk_limits.find(vars[v]);
      const double configured = (it != jerk_limits.end()) ? it->second : 0.0;
      jerk_limits[vars[v]] = std::max(configured, peak * seed_jerk_margin_);
      RCLCPP_DEBUG(rclcpp::get_logger("a3_trajectory_processing"),
                   "jerk %s: configured=%.3f seed_demand=%.3f -> %.3f", vars[v].c_str(), configured, peak,
                   jerk_limits[vars[v]]);
    }
  }

  static std::unordered_map<std::string, double>
  scale_map(const std::unordered_map<std::string, double>& base, double factor)
  {
    auto out = base;
    for (auto& kv : out)
    {
      kv.second *= factor;
    }
    return out;
  }

  static trajectory_msgs::msg::JointTrajectory
  to_msg(const robot_trajectory::RobotTrajectory& traj, const std::vector<std::string>& joint_names)
  {
    const auto* group = traj.getGroup();
    const auto& active_joints = group->getActiveJointModels();
    trajectory_msgs::msg::JointTrajectory out;
    out.joint_names = joint_names;
    out.points.reserve(traj.getWayPointCount());
    for (size_t i = 0; i < traj.getWayPointCount(); ++i)
    {
      const auto& state = traj.getWayPoint(i);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.reserve(joint_names.size());
      point.velocities.reserve(joint_names.size());
      point.accelerations.reserve(joint_names.size());
      for (size_t j = 0; j < active_joints.size(); ++j)
      {
        point.positions.push_back(state.getJointPositions(active_joints[j])[0]);
        point.velocities.push_back(state.getJointVelocities(active_joints[j])[0]);
        point.accelerations.push_back(state.getJointAccelerations(active_joints[j])[0]);
      }
      point.time_from_start =
          rclcpp::Duration::from_seconds(traj.getWayPointDurationFromStart(i));
      out.points.push_back(std::move(point));
    }
    return out;
  }

  bool build_limits(const std::vector<std::string>& joint_names, double v_scaling, double a_scaling,
                    std::unordered_map<std::string, double>& velocity_limits,
                    std::unordered_map<std::string, double>& acceleration_limits,
                    std::unordered_map<std::string, double>& jerk_limits)
  {
    for (const auto& name : joint_names)
    {
      const std::string v_param = "velocity_limits." + name;
      const std::string a_param = "acceleration_limits." + name;
      const std::string j_param = "jerk_limits." + name;

      if (!has_parameter(v_param) || !has_parameter(a_param))
      {
        RCLCPP_ERROR(get_logger(), "missing limit parameter for joint '%s' (%s / %s)", name.c_str(),
                     v_param.c_str(), a_param.c_str());
        return false;
      }
      const double v = get_parameter(v_param).as_double() * v_scaling;
      const double a = get_parameter(a_param).as_double() * a_scaling;
      if (!(v > 0.0) || !(a > 0.0))
      {
        RCLCPP_ERROR(get_logger(), "non-positive limit for joint '%s' (v=%f a=%f)", name.c_str(), v, a);
        return false;
      }
      velocity_limits[name] = v;
      acceleration_limits[name] = a;
      if (has_parameter(j_param))
      {
        const double j = get_parameter(j_param).as_double();
        jerk_limits[name] = (j > 0.0) ? j : default_jerk_scale_ * a;
      }
      else
      {
        jerk_limits[name] = default_jerk_scale_ * a;
      }
    }
    return true;
  }

  std::string group_name_;
  double default_jerk_scale_;
  double seed_jerk_margin_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
  rclcpp::Service<a3_msgs::srv::RetimeTrajectory>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<RetimeTrajectoryNode>(options);
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
