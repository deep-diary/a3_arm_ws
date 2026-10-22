// F180: servo 绝对目标（外部积分）+ Ruckig 在线 OTG。
//
// 背景（见 docs/edge/requirements/F180-servo-absolute-target-integrator.md）：
// moveit_servo 每周期 target = 实测 + Δθ，运动期目标只领先 1 个 tick（0.1 m/s 档 1mm≈1mrad），
// MIT 位置环 kp·e ≈ 0.08 Nm ≪ 抗重力 2.4 Nm → 重力在运动中获胜（x 一动 z 就掉），
// 且命令流逐周期 CV 0.16（震动）。本节点复用同一条 twist 输入，改成：
//
//   twist ──积分──> 绝对末端目标 T_target（z 锁 / 工作空间 / 单周期步长夹紧）
//        ──IK───>  q_goal（复用 MoveIt RobotState::setFromIK = 本仓 pick_ik 配置）
//        ──Ruckig OTG（start=实测 q/q̇, target=q_goal, target_velocity=J⁻¹·v_cmd）──>
//        ──> /arm_controller/joint_trajectory（单点，time_from_start=publish_period）
//
// 与 moveit_servo 的差异只有两点：**目标绝对**（twist=0 时目标不动 → 保持位姿/抗外力），
// 输出经 **jerk-limited OTG（C² 连续）** → 没有 20 ms 跳变与 spline 二阶项跳变。
// 关节限位/速度上限/门禁/急停仍由原有安全层负责（本节点另按 Ruckig 限幅自我约束）。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ruckig/ruckig.hpp>

namespace
{
constexpr size_t kDof = 6;  // arm 组恒为 L1..L6（夹爪是独立组）
using Otg = ruckig::Ruckig<kDof>;
using OtgInput = ruckig::InputParameter<kDof>;
using OtgTraj = ruckig::Trajectory<kDof>;

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}  // namespace

class ServoTargetNode : public rclcpp::Node
{
public:
  ServoTargetNode() : rclcpp::Node("a3_servo_target")
  {
    group_name_ = declare_parameter<std::string>("group_name", "arm");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tip_frame_ = declare_parameter<std::string>("tip_frame", "end_effector");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");
    out_topic_ = declare_parameter<std::string>("out_topic", "/arm_controller/joint_trajectory");
    js_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    dt_ = declare_parameter<double>("publish_period", 0.01);
    hold_z_ = declare_parameter<bool>("hold_z", true);
    max_step_trans_ = declare_parameter<double>("max_step_trans", 0.02);  // [m]/周期
    twist_timeout_ = declare_parameter<double>("twist_timeout", 0.30);    // [s]
    ik_timeout_ = declare_parameter<double>("ik_timeout", 0.005);         // [s] 须 < publish_period
    mode_topic_ = declare_parameter<std::string>("control_mode_topic", "/a3/control_mode");
    mode_topic_timeout_ = declare_parameter<double>("control_mode_timeout", 0.5);
    max_vel_ = declare_parameter<double>("ruckig_max_velocity", 1.5);     // [rad/s]
    max_acc_ = declare_parameter<double>("ruckig_max_acceleration", 5.0);
    max_jerk_ = declare_parameter<double>("ruckig_max_jerk", 50.0);
    max_ik_jump_ = declare_parameter<double>("max_ik_jump", 0.05);        // [rad]/周期
    // CLIK（默认）：阻尼最小二乘，种子=上一拍解 → 解连续
    use_clik_ = (declare_parameter<std::string>("ik_solver", "clik") == "clik");
    clik_iters_ = declare_parameter<int>("clik_iters", 12);
    clik_damping_ = declare_parameter<double>("clik_damping", 0.02);
    clik_tol_pos_ = declare_parameter<double>("clik_tol_pos", 3e-4);      // [m]
    clik_tol_rot_ = declare_parameter<double>("clik_tol_rot", 1e-3);      // [rad]
    clik_max_dq_ = declare_parameter<double>("clik_max_dq", 0.15);        // [rad]/迭代
    // IK 解低通时间常数：pick_ik 在 position_threshold(1e-4) 容差内的落点逐帧抖动约 0.01 rad，
    // 折成速度是 ~1 rad/s 的"假速度"；直接喂 OTG 会放大成 1.3Hz 的 0↔0.45rad/s 摆振（实测 L1/L5）。
    goal_filter_tau_ = declare_parameter<double>("goal_filter_tau", 0.06);  // [s]，0=关闭
    // 跟踪误差保护阈值：**必须大于正常工作时的领先量**（L1 约 8.6mm/rad，30mm/s 下实测领先 0.6~1.1rad）。
    // 设成 0.30 会每 ~0.7s 误触发一次"目标重锚"→ 绝对目标被打回实测 → 冲一下/退回/再冲 = "一快一慢"（实测）。
    max_track_err_ = declare_parameter<double>("max_track_err", 3.0);      // [rad]
    // OTG 前瞻：目标只领先 1 个周期(~0.002rad)时，配非零 target_velocity 是病态的
    // （加速/减速/反转每帧重选 → 输出 ±0.25rad/s 摆振）。给 0.06~0.1s 的前瞻让 OTG 真正"巡航"。
    lookahead_ = declare_parameter<double>("lookahead", 0.08);             // [s]，0=关闭
    // vj 一阶低通：J⁻¹ 对 q_goal 的亚毫弧度抖动敏感，实测 vj 逐帧抖 23%；
    // 直接当 OTG 的 target_velocity 会让它每帧重选加/减速 → 输出摆振。
    vj_filter_tau_ = declare_parameter<double>("vj_filter_tau", 0.05);     // [s]，0=关闭
    pos_gain_ = declare_parameter<double>("pos_gain", 4.0);               // [1/s] 位置闭环增益
    // 加速度环增益：a_des = acc_gain*(v_ref - v)。
    // **必须不能写成 (v_ref - v)/dt**：那是无差拍指令，再叠加 jerk 限幅（qa 每拍最多变
    // max_jerk*dt）后等效为纯积分，实测在静止时形成 2Hz、±0.35rad/s 的极限环（L1/L5 反相摆动）。
    // 取 20 1/s：ωn=√(Kp·Ka)≈8.9rad/s，ζ=√Ka/(2√Kp)≈1.1 → 过阻尼二阶，无自激。
    acc_gain_ = declare_parameter<double>("acc_gain", 20.0);              // [1/s]
    ws_min_ = declare_parameter<std::vector<double>>("workspace_min", { -0.60, -0.45, 0.02 });
    ws_max_ = declare_parameter<std::vector<double>>("workspace_max", { 0.45, 0.45, 0.70 });
    log_period_ = declare_parameter<double>("log_period", 1.0);
    debug_csv_ = declare_parameter<std::string>("debug_csv", "");  // 非空则 100Hz 落调试 CSV
    if (!debug_csv_.empty()) {
      dbg_.open(debug_csv_);
      if (dbg_.is_open()) {
        dbg_ << std::setprecision(17);
        dbg_ << "t,tx,ty,tz,eex,eey,eez,track_mm";
        for (size_t i = 0; i < kDof; ++i) dbg_ << ",g" << i;
        for (size_t i = 0; i < kDof; ++i) dbg_ << ",c" << i;
        for (size_t i = 0; i < kDof; ++i) dbg_ << ",vj" << i;
        for (size_t i = 0; i < kDof; ++i) dbg_ << ",qd" << i;
        for (size_t i = 0; i < kDof; ++i) dbg_ << ",vf" << i;
        dbg_ << ",ik,otg\n";
        RCLCPP_INFO(get_logger(), "调试 CSV: %s", debug_csv_.c_str());
      }
    }
    if (ws_min_.size() != 3 || ws_max_.size() != 3) {
      throw std::runtime_error("workspace_min/max 必须是 3 元数组");
    }
  }

  // 模型与 ROS I/O 必须在 shared_from_this() 可用之后建立（构造期调用会 bad_weak_ptr）
  void init()
  {
    robot_model_loader::RobotModelLoader::Options opts;
    opts.robot_description_ = "robot_description";
    opts.load_kinematics_solvers_ = true;
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      this->shared_from_this(), opts);
    model_ = loader_->getModel();
    if (!model_) {
      RCLCPP_FATAL(get_logger(), "RobotModel 加载失败（缺 robot_description？）");
      throw std::runtime_error("no robot model");
    }
    jmg_ = model_->getJointModelGroup(group_name_);
    if (!jmg_ || jmg_->getActiveJointModelNames().size() != kDof) {
      RCLCPP_FATAL(get_logger(), "组 '%s' 不存在或活跃关节数 != %zu", group_name_.c_str(), kDof);
      throw std::runtime_error("bad group");
    }
    joint_names_ = jmg_->getActiveJointModelNames();
    joint_lo_.resize(kDof);
    joint_hi_.resize(kDof);
    for (size_t i = 0; i < kDof; ++i) {
      const auto* jm = jmg_->getActiveJointModels()[i];
      if (jm->getVariableCount() == 1) {
        const auto& b = model_->getVariableBounds(jm->getVariableNames()[0]);
        joint_lo_[i] = b.min_position_;
        joint_hi_[i] = b.max_position_;
      } else {
        joint_lo_[i] = -M_PI;
        joint_hi_[i] = M_PI;
      }
    }
    state_ = std::make_unique<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();

    otg_ = std::make_unique<Otg>(dt_);

    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      js_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(msg); });
    sub_tw_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { onTwist(msg); });
    // F21/F65 模式互锁：只在 SERVO 模式下发命令；goto/回放(TRAJ_RUNNING)时本节点必须闭嘴，
    // 否则 100Hz 单点帧会和编排层的轨迹在同一话题上互相覆盖（实测 goto home/idle 失效）。
    sub_mode_ = create_subscription<std_msgs::msg::String>(
      mode_topic_, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_);
        mode_ = msg->data;
        mode_t_ = now();
      });
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(out_topic_, rclcpp::QoS(10));
    srv_reanchor_ = create_service<std_srvs::srv::Trigger>(
      "~/reanchor",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!have_state_) {
          resp->success = false;
          resp->message = "no joint state yet";
          return;
        }
        target_ = state_->getGlobalLinkTransform(tip_frame_);
        z_lock_ = target_.translation().z();
        q_goal_ = q_meas_;
        resp->success = true;
        resp->message = "reanchored to measured";
      });

    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { step(); });
    RCLCPP_INFO(get_logger(),
                "a3_servo_target: group=%s dt=%.3fs hold_z=%d out=%s twist=%s",
                group_name_.c_str(), dt_, hold_z_ ? 1 : 0, out_topic_.c_str(), twist_topic_.c_str());
  }

private:
  // 阻尼最小二乘 CLIK：从 q 出发迭代逼近 target（tip 位姿，base 系）。
  // 种子是上一拍解 → 输出天然连续；限步 + 关节限位；失败返回 false（调用方保持上一解）。
  bool clikSolve(const Eigen::Isometry3d& target, std::vector<double>& q)
  {
    moveit::core::RobotState st = *state_;
    st.setJointGroupPositions(jmg_, q);
    st.updateLinkTransforms();
    bool ok = false;
    for (int it = 0; it < clik_iters_; ++it) {
      const Eigen::Isometry3d cur = st.getGlobalLinkTransform(tip_frame_);
      Eigen::Matrix<double, 6, 1> err;
      err.head<3>() = target.translation() - cur.translation();
      const Eigen::AngleAxisd aa(cur.linear().transpose() * target.linear());
      // 注意：aa 的轴在**tip 系**，而 getJacobian 的角分量在 **base 系** → 必须转到 base 系，
      // 否则迭代方向错、永远不收敛（实测 ik_fail 4010/4583）。
      err.tail<3>() = cur.linear() * (aa.axis() * aa.angle());
      if (err.head<3>().norm() < clik_tol_pos_ && err.tail<3>().norm() < clik_tol_rot_) {
        ok = true;
        break;
      }
      const Eigen::MatrixXd J = st.getJacobian(jmg_);
      const Eigen::Matrix<double, 6, 6> JJt =
        J * J.transpose() + clik_damping_ * clik_damping_ * Eigen::Matrix<double, 6, 6>::Identity();
      const Eigen::Matrix<double, kDof, 1> dq = J.transpose() * JJt.ldlt().solve(err);
      for (size_t i = 0; i < kDof; ++i) {
        const double d = clampd(dq[i], -clik_max_dq_, clik_max_dq_);
        q[i] = clampd(q[i] + d, joint_lo_[i], joint_hi_[i]);
      }
      st.setJointGroupPositions(jmg_, q);
      st.updateLinkTransforms();
    }
    return ok;
  }
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mu_);
    std::vector<double> q(kDof, 0.0), qd(kDof, 0.0);
    for (size_t i = 0; i < kDof; ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) return;  // 不完整帧丢弃
      const size_t k = static_cast<size_t>(it - msg->name.begin());
      if (k < msg->position.size()) q[i] = msg->position[k];
      if (k < msg->velocity.size()) qd[i] = msg->velocity[k];
    }
    state_->setJointGroupPositions(jmg_, q);
    state_->setJointGroupVelocities(jmg_, qd);
    if (!have_state_) {
      target_ = state_->getGlobalLinkTransform(tip_frame_);
      z_lock_ = target_.translation().z();
      q_goal_ = q;
      for (size_t i = 0; i < kDof; ++i) { q_cmd_[i] = q[i]; qd_cmd_[i] = qd[i]; qa_cmd_[i] = 0.0; }
      have_state_ = true;
      RCLCPP_INFO(get_logger(), "首帧状态就绪：绝对目标锚定 z=%.4f", z_lock_);
    }
    q_meas_ = q;
    qd_meas_ = qd;
  }

  void onTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mu_);
    twist_ = *msg;
    have_twist_ = true;
    twist_t_ = now();
  }

  void step()
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!have_state_) return;

    // ---- 0) 模式互锁（F21/F119）----
    // /a3/control_mode 是**多写者槽位**：servo_mode_bridge 写 SERVO，而 gravity_torque_node /
    // follow_joint_trajectory_action 周期性写 IDLE/READY → 值以 ~17Hz 交替。
    // 因此绝不能按 "值 == SERVO" 门控（实测一次会话误退出 34 次，每次都停发+重锚 = "隔几秒顿一下"）。
    // 判据改为：① 有新鲜 twist；② 不处于编排层的轨迹/异常态（那些必须让路）。
    const bool mode_blocked = (mode_ == "TRAJ_RUNNING" || mode_ == "READY" || mode_ == "FAULT" ||
                              mode_ == "ZERO_TORQUE" || mode_ == "COOLING" || mode_ == "DISABLED");
    const bool twist_fresh = have_twist_ && (now() - twist_t_).seconds() < twist_timeout_;
    const bool active = !mode_blocked && twist_fresh;
    if (!active) {
      last_step_t_ = now();
      target_ = state_->getGlobalLinkTransform(tip_frame_);
      z_lock_ = target_.translation().z();
      q_goal_ = q_meas_;
      for (size_t i = 0; i < kDof; ++i) {
        q_cmd_[i] = q_meas_[i];
        qd_cmd_[i] = qd_meas_[i];
        qa_cmd_[i] = 0.0;
      }
      if (was_in_servo_) {
        was_in_servo_ = false;
        RCLCPP_INFO(get_logger(), "停止下发（mode=%s twist_fresh=%d）→ 重锚实测",
                    mode_.c_str(), twist_fresh ? 1 : 0);
      }
      return;
    }
    if (!was_in_servo_) {
      was_in_servo_ = true;
      last_step_t_ = now();
      RCLCPP_INFO(get_logger(), "开始下发（mode=%s）→ 以当前实测位姿为绝对目标起点", mode_.c_str());
    }

    // 实际步长：定时器在 100Hz 上有抖动（实测偶发 2.8ms 连发两拍、17ms 迟一拍）。
    // 若仍按固定 10ms 积分，连发时会把 10ms 的位移在 3ms 内推出去（命令瞬时速度翻倍），
    // JTC 收到 3ms 间隔、各自 10ms 时长的单点帧 → 末端一抖。这里用实际间隔并夹紧到
    // [0.5dt, 2dt]，同时把 time_from_start 设成同一值，保证位置/时间一致。
    const rclcpp::Time t_now = now();
    double dt_step = dt_;
    if (last_step_t_.nanoseconds() > 0) {
      const double d = (t_now - last_step_t_).seconds();
      if (d > 0.0) dt_step = clampd(d, 0.5 * dt_, 2.0 * dt_);
    }
    last_step_t_ = t_now;

    // ---- 1) 指令有效性（超时 = 零速：目标不动 = 绝对保持）----
    Eigen::Matrix<double, 6, 1> v = Eigen::Matrix<double, 6, 1>::Zero();
    if (have_twist_ && (now() - twist_t_).seconds() < twist_timeout_) {
      v << twist_.twist.linear.x, twist_.twist.linear.y, twist_.twist.linear.z,
        twist_.twist.angular.x, twist_.twist.angular.y, twist_.twist.angular.z;
    }
    const bool moving = v.head<3>().norm() > 1e-6 || v.tail<3>().norm() > 1e-6;

    // ---- 2) 绝对目标积分（twist 在 base 系 → 左乘）----
    if (moving) {
      Eigen::Isometry3d d = Eigen::Isometry3d::Identity();
      const Eigen::Vector3d w = v.tail<3>();
      const double th = w.norm();
      if (th > 1e-9) d.linear() = Eigen::AngleAxisd(th, w / th).toRotationMatrix();
      d.translation() = v.head<3>() * dt_step;
      Eigen::Isometry3d cand = d * target_;
      // 单周期步长夹紧
      Eigen::Vector3d dp = cand.translation() - target_.translation();
      const double n = dp.norm();
      if (n > max_step_trans_ && n > 1e-9) dp *= max_step_trans_ / n;
      cand.translation() = target_.translation() + dp;
      // 工作空间夹紧
      for (int i = 0; i < 3; ++i) {
        cand.translation()[i] = clampd(cand.translation()[i], ws_min_[i], ws_max_[i]);
      }
      // z 锁：没有 z 指令时锁在当前高度；有 z 指令时跟随并更新锁点
      if (hold_z_ && std::abs(v[2]) < 1e-6) {
        cand.translation().z() = z_lock_;
      } else {
        z_lock_ = cand.translation().z();
      }
      target_ = cand;
    }

    // ---- 3) IK ----
    // 两条路：
    //  ik_solver="clik"  : 自写阻尼 CLIK（种子=上一拍解，限步+限位）→ **解连续**，这是跟踪要的；
    //  ik_solver="moveit": 走 pick_ik（RobotState::setFromIK）。
    // 为什么默认 CLIK：pick_ik 在 1e-4 容差内落点不唯一、在 y 大偏摆位形还会换支；
    // 实测同一目标逐帧跳 0.05~0.3rad，放大到命令流就是"一快一慢"（x 轴修好后 CV 0.023，y 轴直接卡死）。
    bool ik_ok = false;
    std::vector<double> qsol;
    if (use_clik_) {
      qsol = q_goal_;
      ik_ok = clikSolve(target_, qsol);
    } else {
      moveit::core::RobotState ik_state = *state_;
      ik_state.setJointGroupPositions(jmg_, q_goal_);
      ik_ok = ik_state.setFromIK(jmg_, target_, tip_frame_, ik_timeout_);
      if (ik_ok) ik_state.copyJointGroupPositions(jmg_, qsol);
    }
    if (ik_ok) {
      double dmax = 0.0;
      for (size_t i = 0; i < kDof; ++i) dmax = std::max(dmax, std::abs(qsol[i] - q_goal_[i]));
      if (dmax < max_ik_jump_) {  // 防换支
        if (goal_filter_tau_ > 1e-3) {
          const double a = std::min(1.0, dt_ / goal_filter_tau_);
          for (size_t i = 0; i < kDof; ++i) q_goal_[i] += a * (qsol[i] - q_goal_[i]);
        } else {
          q_goal_ = qsol;
        }
        ik_fail_streak_ = 0;
      } else {
        ++ik_jump_reject_;
      }
    } else {
      ++ik_fail_streak_;
      ++ik_fail_total_;
    }

    // ---- 4) 目标速度前馈 J⁻¹·v（关键：否则远目标每周期都在刹车）----
    std::array<double, kDof> vj{};
    if (moving) {
      // 前馈用**命令位姿**的雅可比（q_goal 已低通），比用实测更平滑，避免量化噪声进前馈
      moveit::core::RobotState fk_state = *state_;
      fk_state.setJointGroupPositions(jmg_, q_goal_);
      fk_state.updateLinkTransforms();
      const Eigen::MatrixXd J = fk_state.getJacobian(jmg_);  // 6×6，base 系 / tip 参考点
      Eigen::Matrix<double, 6, 1> vv = v;
      Eigen::Matrix<double, kDof, 1> qd_ff = J.completeOrthogonalDecomposition().solve(vv);
      for (size_t i = 0; i < kDof; ++i) vj[i] = clampd(qd_ff[i], -max_vel_, max_vel_);
    }
    // vj 低通
    {
      const double a = (vj_filter_tau_ > 1e-3) ? std::min(1.0, dt_ / vj_filter_tau_) : 1.0;
      for (size_t i = 0; i < kDof; ++i) vj_f_[i] += a * (vj[i] - vj_f_[i]);
    }

    // ---- 5) 有状态速度积分器（前馈 + 位置闭环 + 加速度/加加速度限幅）----
    // 为什么不用"每拍 Ruckig 重规划"：目标只领先 1 个周期时，Ruckig 的时间最优解对
    // 目标的亚毫弧度抖动极其敏感（实测 IK 落点抖 0.5mrad → 输出 ±150% 速度摆振、
    // 甚至反向）。有状态积分器对同样的抖动只产生 Kp*0.5mrad 的响应，结构上不会自激；
    // 加速度/加加速度仍按 max_acc/max_jerk 限幅，输出照样是 C² 连续。
    const double a_step = max_jerk_ * dt_step;  // 每拍允许的加速度变化
    std::array<double, kDof> q_out{}, qd_out{}, qa_out{};
    for (size_t i = 0; i < kDof; ++i) {
      double v_des = vj_f_[i] + pos_gain_ * (q_goal_[i] - q_cmd_[i]);
      v_des = clampd(v_des, -max_vel_, max_vel_);
      const double a_des = clampd(acc_gain_ * (v_des - qd_cmd_[i]), -max_acc_, max_acc_);
      qa_cmd_[i] = clampd(a_des, qa_cmd_[i] - a_step, qa_cmd_[i] + a_step);
      qd_cmd_[i] = clampd(qd_cmd_[i] + qa_cmd_[i] * dt_step, -max_vel_, max_vel_);
      q_cmd_[i] += qd_cmd_[i] * dt_step;
      q_out[i] = q_cmd_[i];
      qd_out[i] = qd_cmd_[i];
      qa_out[i] = qa_cmd_[i];
    }
    const int otg_res = 0;  // 兼容调试 CSV 字段

    // ---- 5b) 安全：命令跑得太前（被挡/跟不上）→ 命令状态回拉到实测并重锚 ----
    double track_err = 0.0;
    for (size_t i = 0; i < kDof; ++i) {
      track_err = std::max(track_err, std::abs(q_cmd_[i] - q_meas_[i]));
    }
    if (track_err > max_track_err_) {
      ++track_err_resets_;
      for (size_t i = 0; i < kDof; ++i) {
        q_cmd_[i] = q_meas_[i];
        qd_cmd_[i] = qd_meas_[i];
        qa_cmd_[i] = 0.0;
      }
      target_ = state_->getGlobalLinkTransform(tip_frame_);
      z_lock_ = target_.translation().z();
      q_goal_ = q_meas_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "跟踪误差 %.2f rad 超限 → 命令重锚实测（累计 %d 次）",
                           track_err, track_err_resets_);
    }

    // ---- 6) 发布单点帧（time_from_start = dt）----
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);  // 0 = 立即
    msg.header.frame_id = base_frame_;
    msg.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(q_out.begin(), q_out.end());
    pt.velocities.assign(qd_out.begin(), qd_out.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(dt_step);
    msg.points.push_back(pt);
    pub_->publish(msg);
    published_ = true;

    if (dbg_.is_open()) {
      const auto &ee = state_->getGlobalLinkTransform(tip_frame_).translation();
      dbg_ << now().nanoseconds() * 1e-9 << ',' << target_.translation().x() << ',' << target_.translation().y()
           << ',' << target_.translation().z() << ',' << ee.x() << ',' << ee.y() << ',' << ee.z() << ','
           << (target_.translation() - ee).norm() * 1e3;
      for (size_t i = 0; i < kDof; ++i) dbg_ << ',' << q_goal_[i];
      for (size_t i = 0; i < kDof; ++i) dbg_ << ',' << q_out[i];
      for (size_t i = 0; i < kDof; ++i) dbg_ << ',' << vj[i];
      for (size_t i = 0; i < kDof; ++i) dbg_ << ',' << qd_out[i];
      for (size_t i = 0; i < kDof; ++i) dbg_ << ',' << vj_f_[i];
      dbg_ << ',' << (ik_ok ? 1 : 0) << ',' << otg_res << '\n';
    }

    // ---- 7) 低频诊断 ----
    const auto tnow = now();
    if ((tnow - last_log_t_).seconds() >= log_period_) {
      last_log_t_ = tnow;
      const Eigen::Vector3d err =
        target_.translation() - state_->getGlobalLinkTransform(tip_frame_).translation();
      RCLCPP_INFO(get_logger(),
                  "err=%.1fmm z=%.4f ik_fail=%d/%d ik_jump_rej=%d otg_fail=%d moving=%d |v|=%.3f",
                  err.norm() * 1e3, target_.translation().z(), ik_fail_streak_, ik_fail_total_,
                  ik_jump_reject_, otg_fail_total_, moving ? 1 : 0, v.head<3>().norm());
    }
  }

  // params
  std::string group_name_, base_frame_, tip_frame_, twist_topic_, out_topic_, js_topic_;
  std::string mode_topic_;
  double mode_topic_timeout_{ 0.5 };
  double dt_{ 0.01 }, max_step_trans_{ 0.02 }, twist_timeout_{ 0.3 }, ik_timeout_{ 0.02 };
  double max_vel_{ 1.5 }, max_acc_{ 8.0 }, max_jerk_{ 200.0 }, max_ik_jump_{ 0.05 };
  bool use_clik_{ true };
  int clik_iters_{ 8 };
  double clik_damping_{ 0.02 }, clik_tol_pos_{ 1e-4 }, clik_tol_rot_{ 1e-4 }, clik_max_dq_{ 0.15 };
  std::vector<double> joint_lo_, joint_hi_;
  double goal_filter_tau_{ 0.06 };
  double log_period_{ 1.0 };
  std::string debug_csv_;
  std::ofstream dbg_;
  bool hold_z_{ true };
  std::vector<double> ws_min_, ws_max_;

  // model
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_{ nullptr };
  std::vector<std::string> joint_names_;
  std::unique_ptr<moveit::core::RobotState> state_;
  std::unique_ptr<Otg> otg_;

  // ros
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_tw_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reanchor_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  std::mutex mu_;
  bool have_state_{ false }, have_twist_{ false }, published_{ false }, was_in_servo_{ false };
  std::string mode_;
  rclcpp::Time mode_t_{ 0, 0, RCL_ROS_TIME };
  std::vector<double> q_meas_, qd_meas_, q_goal_;
  std::array<double, kDof> q_cmd_{}, qd_cmd_{}, qa_cmd_{};
  double max_track_err_{ 3.0 };  // [rad] 命令-实测偏差上限（只在真的被挡/失速时才重锚）
  double lookahead_{ 0.08 };     // [s] OTG 前瞻
  double vj_filter_tau_{ 0.05 };
  double pos_gain_{ 4.0 };
  double acc_gain_{ 20.0 };
  std::array<double, kDof> vj_f_{};
  int track_err_resets_{ 0 };
  geometry_msgs::msg::TwistStamped twist_;
  rclcpp::Time twist_t_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_log_t_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_step_t_{ 0, 0, RCL_ROS_TIME };
  Eigen::Isometry3d target_{ Eigen::Isometry3d::Identity() };
  double z_lock_{ 0.0 };
  int ik_fail_streak_{ 0 }, ik_fail_total_{ 0 }, ik_jump_reject_{ 0 }, otg_fail_total_{ 0 };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoTargetNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
