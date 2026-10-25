// F180/F181: servo 绝对目标（外部积分）+ CLIK + 有状态速度积分。
//
// 背景（见 docs/edge/requirements/F180-servo-absolute-target-integrator.md、F181-...）：
// moveit_servo 每周期 target = 实测 + Δθ，运动期目标只领先 1 个 tick（0.1 m/s 档 1mm≈1mrad），
// MIT 位置环 kp·e ≈ 0.08 Nm ≪ 抗重力 2.4 Nm → 重力在运动中获胜（x 一动 z 就掉）。
// 本节点复用同一条 twist 输入，改成：
//
//   twist ──限幅/低通──> v（F181：模长上限 + 逐轴加速度限幅 + 一阶低通）──积分──>
//         绝对末端目标 T_target（z 锁 / 工作空间 / 单周期步长夹紧）
//        ──CLIK──> q_goal（阻尼最小二乘，种子=上一拍解，解连续）
//        ──有状态加速度环──> /arm_controller/joint_trajectory（单点，time_from_start=实际步长）
//
// 与 moveit_servo 的差异：**目标绝对**（twist=0 时目标经平滑减速后停住 → 保持位姿/抗外力），
// 输出经 acc/jerk 限幅（C² 连续）。关节限位/速度上限/门禁/急停仍由原有安全层负责。
//
// 为什么先限加速度再低通：twist 从 +v 直接跳到 0 / −v 时，目标速度是阶跃；只靠下游关节级
// max_acc/max_jerk 兜底会让位置误差瞬间变大（过冲/抖动）。在 twist 层先做有限加速度的
// 减速/反向段，再一阶低通，目标位置就始终 C¹；代价是松手后目标多走 v·(v/a + τ) ≈ 3–4mm。

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
#include <a3_msgs/msg/arm_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace
{
constexpr size_t kDof = 6;  // arm 组恒为 L1..L6（夹爪是独立组）

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

// 只缩不放大：把向量模长夹到 max_norm
void capNorm(Eigen::Vector3d& v, double max_norm)
{
  const double n = v.norm();
  if (n > max_norm && n > 1e-12) v *= max_norm / n;
}
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
    // 角速度语义：true = 绕末端点旋转（姿态变、末端位置不变）；false = 旧行为（绕基座原点画弧）
    rotation_about_tip_ = declare_parameter<bool>("rotation_about_tip", true);
    max_step_trans_ = declare_parameter<double>("max_step_trans", 0.02);  // [m]/周期
    twist_timeout_ = declare_parameter<double>("twist_timeout", 0.30);    // [s]
    ik_timeout_ = declare_parameter<double>("ik_timeout", 0.005);         // [s] 须 < publish_period
    mode_topic_ = declare_parameter<std::string>("control_mode_topic", "/a3/control_mode");
    stale_topic_ = declare_parameter<std::string>("stale_topic", "/a3/hardware/feedback_stale");
    mode_topic_timeout_ = declare_parameter<double>("control_mode_timeout", 0.5);
    max_vel_ = declare_parameter<double>("max_joint_velocity", 1.5);      // [rad/s]
    max_acc_ = declare_parameter<double>("max_joint_acceleration", 5.0);  // [rad/s²]
    max_jerk_ = declare_parameter<double>("max_joint_jerk", 50.0);        // [rad/s³]
    // F187（伺服侧）：命令状态 q_cmd_ 必须同时夹在 URDF 限位内。
    // 2026-10-24 事故：q_cmd_ 只夹速度不夹位置，vj 前馈把它一路积分到限位外
    // （黑匣子：L4 命令 −1.48 vs 限位 −1.047、L5 +1.78 vs 1.5708），MIT 位置环按
    // kp·(cmd−meas) 顶死 → 堵转 0x10 → 拉断 L5 连杆。
    // 停止下发时的平滑归位：跟踪期间命令一直"领先"实测（正是这份领先在托着臂抗重力），
    // 关跟踪瞬间把命令撤到实测位姿会让臂"掉一下"。改成按 settle_rate 匀速收到实测位姿再停发。
    settle_rate_ = declare_parameter<double>("settle_rate", 0.3);            // [rad/s]
    cmd_limit_margin_ = declare_parameter<double>("cmd_limit_margin", 0.02);  // [rad]
    max_ik_jump_ = declare_parameter<double>("max_ik_jump", 0.05);        // [rad]/周期
    // IK 解与当前种子相差超过 max_ik_jump 时：true=限步逼近（推荐，避免死锁）；false=整帧丢弃（旧行为）
    ik_jump_clip_ = declare_parameter<bool>("ik_jump_clip", true);
    max_ik_jump_hard_ = declare_parameter<double>("max_ik_jump_hard", 0.60);  // 超过则判为真换支
    // IK 解相对**实测关节**的最大允许偏离：超过即认为是姿态等价的另一分支（腕部伸直时常见）
    max_ik_jump_meas_ = declare_parameter<double>("max_ik_jump_meas", 0.20);  // [rad]
    // F181 twist 平滑：模长上限 → 逐轴加速度限幅 → 一阶低通
    twist_max_linear_ = declare_parameter<double>("twist_max_linear", 0.40);      // [m/s]
    twist_max_angular_ = declare_parameter<double>("twist_max_angular", 1.2);     // [rad/s]
    twist_max_accel_ = declare_parameter<double>("twist_max_accel", 1.0);         // [m/s²]
    twist_max_ang_accel_ = declare_parameter<double>("twist_max_ang_accel", 3.0); // [rad/s²]
    twist_lpf_tau_ = declare_parameter<double>("twist_lpf_tau", 0.06);            // [s]，≤1e-3 关闭
    // CLIK（默认）：阻尼最小二乘，种子=上一拍解 → 解连续
    use_clik_ = (declare_parameter<std::string>("ik_solver", "clik") == "clik");
    clik_iters_ = declare_parameter<int>("clik_iters", 12);
    clik_damping_ = declare_parameter<double>("clik_damping", 0.02);
    clik_tol_pos_ = declare_parameter<double>("clik_tol_pos", 3e-4);      // [m]
    clik_tol_rot_ = declare_parameter<double>("clik_tol_rot", 1e-3);      // [rad]
    // 伺服是逐拍小步推进的：允许"还没完全收敛但误差已经很小"的解，否则每拍都判失败 → 目标回退死锁
    clik_tol_pos_loose_ = declare_parameter<double>("clik_tol_pos_loose", 0.005);  // [m]
    clik_tol_rot_loose_ = declare_parameter<double>("clik_tol_rot_loose", 0.02);   // [rad]
    clik_max_dq_ = declare_parameter<double>("clik_max_dq", 0.15);        // [rad]/迭代
    // IK 解低通时间常数：pick_ik 在 position_threshold(1e-4) 容差内的落点逐帧抖动约 0.01 rad，
    // 折成速度是 ~1 rad/s 的"假速度"；不过滤会放大成 1.3Hz 的 0↔0.45rad/s 摆振（实测 L1/L5）。
    goal_filter_tau_ = declare_parameter<double>("goal_filter_tau", 0.06);  // [s]，0=关闭
    // 跟踪误差保护阈值：**必须大于正常工作时的领先量**（L1 约 8.6mm/rad，30mm/s 下实测领先 0.6~1.1rad）。
    // 设成 0.30 会每 ~0.7s 误触发一次"目标重锚"→ 绝对目标被打回实测 → 冲一下/退回/再冲 = "一快一慢"（实测）。
    max_track_err_ = declare_parameter<double>("max_track_err", 3.0);      // [rad]
    // vj 一阶低通：J⁻¹ 对 q_goal 的亚毫弧度抖动敏感，实测 vj 逐帧抖 23%；
    // 不过滤会让速度前馈逐帧跳变。
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
    // 编排层状态（单写者）：goto/回放期间必须让路。
    // 不用 /a3/control_mode 做门控——那是多写者槽位，TRAJ_RUNNING 与 IDLE 以十几毫秒交替，
    // 若把 IDLE 当“中性”，goto 刚开始 1.5ms 后本节点就恢复 100Hz 下发，会把 goto 轨迹顶掉
    // （实测 goto home 10s 停在 idle 不动，sweep 全程 CLIK 失败 1405 次）。
    sub_arm_ = create_subscription<a3_msgs::msg::ArmStatus>(
      "/a3/arm_status", rclcpp::QoS(10),
      [this](const a3_msgs::msg::ArmStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_);
        arm_state_ = msg->state;
        arm_state_t_ = now();
      });
    // F81 硬件看门狗（latched）：任一电机反馈 stale 时插件整臂 freeze-hold，命令被丢弃、
    // 实测冻结。此时若继续积分目标，目标会一路跑飞（实测 L6 掉线 20s → 目标顶到工作空间
    // 边界，恢复瞬间臂猛追 + CLIK 换支/撞限位 → 剧烈震荡）。stale=true 时按“非激活”处理：
    // 每拍把目标/命令重锚到实测，恢复瞬间从当前位姿无缝续上。
    auto stale_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    sub_stale_ = create_subscription<std_msgs::msg::Bool>(
      stale_topic_, stale_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_);
        if (msg->data != feedback_stale_) {
          RCLCPP_WARN(get_logger(), "硬件反馈 stale=%d（%s）", msg->data ? 1 : 0,
                      msg->data ? "暂停跟踪并重锚实测" : "恢复跟踪");
        }
        feedback_stale_ = msg->data;
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
        std::vector<double> qc;
        bool out = false;
        target_ = measuredPose(qc, out);
        z_lock_ = target_.translation().z();
        target_ok_ = target_;
        have_target_ok_ = true;
        q_goal_ = qc;
        resp->success = true;
        resp->message = out ? "reanchored to measured (关节越限，已夹紧)"
                            : "reanchored to measured";
      });

    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { step(); });
    RCLCPP_INFO(get_logger(),
                "a3_servo_target: group=%s dt=%.3fs hold_z=%d rot_about_tip=%d out=%s twist=%s",
                group_name_.c_str(), dt_, hold_z_ ? 1 : 0, rotation_about_tip_ ? 1 : 0,
                out_topic_.c_str(), twist_topic_.c_str());
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
    if (!ok) {
      // 没到严格容差：只要姿态/位置残差已足够小，就当"可用解"接受（伺服逐拍推进，不必拍拍收敛）
      const Eigen::Isometry3d cur = st.getGlobalLinkTransform(tip_frame_);
      const double pe = (target.translation() - cur.translation()).norm();
      const double re = std::abs(Eigen::AngleAxisd(cur.linear().transpose() * target.linear()).angle());
      ok = (pe < clik_tol_pos_loose_ && re < clik_tol_rot_loose_);
    }
    return ok;
  }

  // ---------------- 锚定辅助：实测关节必须先夹到 URDF 限位 ----------------
  // 为什么：重锚若直接用"实测 FK"当目标，一旦某个关节越限，clikSolve 里一夹紧就差
  // 2.6°(0.046 rad) > clik_tol_rot_loose_(0.02)，于是**每拍都 ik_fail**、目标被回退钉死、
  // 整臂一个关节都不动（实测 ik_fail 17150/20158 = 85%，表现为"卡在极限且不尝试转动"）。
  bool clampedMeas(std::vector<double>& qc) const
  {
    qc = q_meas_;
    bool out = false;
    for (size_t i = 0; i < kDof; ++i) {
      const double v = clampd(qc[i], joint_lo_[i], joint_hi_[i]);
      // 报告阈值 5 mrad：零点/编码噪声让 L2 在 0.000±0.001 抖是常态，不该每 2s 刷告警；
      // 但仍照常夹紧（夹紧本身无副作用）。
      if (std::abs(v - qc[i]) > 5e-3) out = true;
      qc[i] = v;
    }
    return out;
  }

  Eigen::Isometry3d fkOf(const std::vector<double>& q)
  {
    moveit::core::RobotState st = *state_;
    st.setJointGroupPositions(jmg_, q);
    st.updateLinkTransforms();
    return st.getGlobalLinkTransform(tip_frame_);
  }

  // 返回可作锚点的实测位姿；qc = 要写回 q_goal_ 的关节（夹紧后），out = 是否发生越限。
  Eigen::Isometry3d measuredPose(std::vector<double>& qc, bool& out)
  {
    out = clampedMeas(qc);
    if (!out) return state_->getGlobalLinkTransform(tip_frame_);
    for (size_t i = 0; i < kDof; ++i) {
      if (std::abs(qc[i] - q_meas_[i]) > 5e-3) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "%s 实测 %.4f 越 URDF 限位 [%.4f, %.4f] → 锚定用夹紧后的位姿",
                             joint_names_[i].c_str(), q_meas_[i], joint_lo_[i], joint_hi_[i]);
      }
    }
    return fkOf(qc);
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
    q_meas_ = q;
    qd_meas_ = qd;
    if (!have_state_) {
      std::vector<double> qc;
      bool out = false;
      target_ = measuredPose(qc, out);
      z_lock_ = target_.translation().z();
      target_ok_ = target_;
      have_target_ok_ = true;
      q_goal_ = qc;
      for (size_t i = 0; i < kDof; ++i) { q_cmd_[i] = qc[i]; qd_cmd_[i] = qd[i]; qa_cmd_[i] = 0.0; }
      have_state_ = true;
      RCLCPP_INFO(get_logger(), "首帧状态就绪：绝对目标锚定 z=%.4f", z_lock_);
    }
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

    // ---- 0) 编排层互锁（F21/F65/F181）----
    // 判据：① 有新鲜 twist；② **编排层状态**（单写者 /a3/arm_status.state）在 READY/SERVO，
    // 且 /a3/control_mode 当前值不是 ZERO_TORQUE（自由拖动的兜底）。
    // 绝不能用 /a3/control_mode 的 TRAJ_RUNNING 来判“轨迹中”：它是多写者槽位，TRAJ_RUNNING 与
    // IDLE 以十几毫秒交替；把 IDLE 当“中性”会让本节点在 goto 开始 1.5ms 后就恢复下发 → 顶掉轨迹。
    const bool state_fresh = arm_state_t_.nanoseconds() > 0 &&
                             (now() - arm_state_t_).seconds() < 1.0;
    const bool state_ok = state_fresh && (arm_state_ == "READY" || arm_state_ == "SERVO");
    const bool mode_blocked = (mode_ == "ZERO_TORQUE");
    const bool twist_fresh = have_twist_ && (now() - twist_t_).seconds() < twist_timeout_;
    const bool active = state_ok && !mode_blocked && !feedback_stale_ && twist_fresh;
    if (!active) {
      const rclcpp::Time t_now = now();
      double dt_step = dt_;
      if (last_step_t_.nanoseconds() > 0) {
        const double d = (t_now - last_step_t_).seconds();
        if (d > 0.0) dt_step = clampd(d, 0.5 * dt_, 2.0 * dt_);
      }
      last_step_t_ = t_now;
      std::vector<double> qc;
      bool out = false;
      target_ = measuredPose(qc, out);
      z_lock_ = target_.translation().z();
      target_ok_ = target_;
      have_target_ok_ = true;
      q_goal_ = qc;
      v_lim_.setZero();  // 平滑器状态清零，避免恢复时带旧速度
      v_lp_.setZero();

      // 只有"只是 twist 断了"（跟踪被关/超时）这一种情况才平滑归位；
      // FAULT / ZERO_TORQUE / stale 时一律不发布，交给下层的保持/失能逻辑。
      const bool can_settle = state_ok && !mode_blocked && !feedback_stale_ && !twist_fresh;
      if (can_settle) {
        if (!settle_active_ && !settle_done_) {
          // 进入时拍快照（夹在软限位内）：之后命令朝快照收敛，臂才不会跟着重力往下走。
          for (size_t i = 0; i < kDof; ++i) {
            settle_target_[i] = clampd(qc[i], joint_lo_[i] + cmd_limit_margin_,
                                       joint_hi_[i] - cmd_limit_margin_);
          }
          settle_active_ = true;
          double gap0 = 0.0;
          for (size_t i = 0; i < kDof; ++i)
            gap0 = std::max(gap0, std::abs(settle_target_[i] - q_cmd_[i]));
          RCLCPP_INFO(get_logger(),
                      "停止下发 → 平滑归位（%.3f rad @ %.2f rad/s，约 %.2f s）",
                      gap0, settle_rate_, gap0 / std::max(1e-3, settle_rate_));
        }
        if (settle_active_) {
          double gap = 0.0;
          for (size_t i = 0; i < kDof; ++i)
            gap = std::max(gap, std::abs(settle_target_[i] - q_cmd_[i]));
          if (gap > 1e-3) {
            const double step = settle_rate_ * dt_step;
            std::array<double, kDof> q_out{}, qd_zero{};
            for (size_t i = 0; i < kDof; ++i) {
              q_cmd_[i] += clampd(settle_target_[i] - q_cmd_[i], -step, step);
              qd_cmd_[i] = 0.0;
              qa_cmd_[i] = 0.0;
              q_out[i] = q_cmd_[i];
            }
            trajectory_msgs::msg::JointTrajectory msg;
            msg.header.stamp = rclcpp::Time(0);
            msg.header.frame_id = base_frame_;
            msg.joint_names = joint_names_;
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.assign(q_out.begin(), q_out.end());
            pt.velocities.assign(qd_zero.begin(), qd_zero.end());
            pt.time_from_start = rclcpp::Duration::from_seconds(dt_step);
            msg.points.push_back(pt);
            pub_->publish(msg);
            was_in_servo_ = false;
            return;
          }
          // 完成标记：否则下面的"重锚实测"会把 q_cmd_ 拉回实测，下一拍 gap 又 >1e-3
          // → 100 Hz 反复"开始/完成"（仍在发同一位置，但日志刷屏、publish 永不停）。
          settle_active_ = false;
          settle_done_ = true;
          RCLCPP_INFO(get_logger(), "平滑归位完成，停止下发");
        }
      }
      settle_active_ = false;
      if (!can_settle) settle_done_ = false;   // FAULT/stale/ZERO_TORQUE 后再停发要重新归位
      for (size_t i = 0; i < kDof; ++i) {
        q_cmd_[i] = qc[i];
        qd_cmd_[i] = qd_meas_[i];
        qa_cmd_[i] = 0.0;
      }
      if (was_in_servo_) {
        was_in_servo_ = false;
        RCLCPP_INFO(get_logger(), "停止下发（state=%s mode=%s stale=%d twist_fresh=%d）→ 重锚实测",
                    arm_state_.c_str(), mode_.c_str(), feedback_stale_ ? 1 : 0, twist_fresh ? 1 : 0);
      }
      return;
    }
    if (!was_in_servo_) {
      was_in_servo_ = true;
      settle_done_ = false;   // 重新收到 twist：下次再停发要重新归位
      last_step_t_ = now();
      RCLCPP_INFO(get_logger(), "开始下发（state=%s）→ 以当前实测位姿为绝对目标起点",
                  arm_state_.c_str());
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

    // ---- 1) 指令有效性 + twist 平滑（F181）----
    // 超时 = 零速（绝对保持）；否则先模长限幅、再逐轴加速度限幅、最后一阶低通。
    Eigen::Matrix<double, 6, 1> v_raw = Eigen::Matrix<double, 6, 1>::Zero();
    if (have_twist_ && (now() - twist_t_).seconds() < twist_timeout_) {
      v_raw << twist_.twist.linear.x, twist_.twist.linear.y, twist_.twist.linear.z,
        twist_.twist.angular.x, twist_.twist.angular.y, twist_.twist.angular.z;
      Eigen::Vector3d lin = v_raw.head<3>();
      Eigen::Vector3d ang = v_raw.tail<3>();
      capNorm(lin, twist_max_linear_);
      capNorm(ang, twist_max_angular_);
      v_raw.head<3>() = lin;
      v_raw.tail<3>() = ang;
    }
    // 逐轴加速度限幅：正→0 / 正→负都变成有限斜率，不再给下游丢速度台阶
    const double a_lin = twist_max_accel_ * dt_step;
    const double a_ang = twist_max_ang_accel_ * dt_step;
    for (size_t i = 0; i < 3; ++i) {
      v_lim_[i] += clampd(v_raw[i] - v_lim_[i], -a_lin, a_lin);
      v_lim_[i + 3] += clampd(v_raw[i + 3] - v_lim_[i + 3], -a_ang, a_ang);
    }
    // 一阶低通（τ=0 时直通）
    if (twist_lpf_tau_ > 1e-3) {
      const double al = std::min(1.0, dt_step / twist_lpf_tau_);
      v_lp_ += al * (v_lim_ - v_lp_);
    } else {
      v_lp_ = v_lim_;
    }
    const Eigen::Matrix<double, 6, 1> v = v_lp_;
    const bool moving = v.head<3>().norm() > 1e-4 || v.tail<3>().norm() > 1e-3;

    // ---- 1b) 已到位且无指令：分支异常时才锚实测，正常保持则不动 ----
    // 腕部接近伸直时 IK 存在姿态等价的两个分支，即使目标静止也会被"限步跟进"一路拖到另一支
    // （实测停发后工具继续转 10°→29°）。所以检测到分支异常后，下一拍把种子/目标换成实测。
    // **不能无条件换**：无条件把 q_goal_ 换成实测 == "实测在哪就保持在哪"，位置环误差恒 ≈0，
    // 重力把臂拉低时命令就逐拍跟着下坠（2026-10-25 实测：无人脸 18 s 内 L3 漂 0.09 rad，
    // 命令−实测恒为 0.000）。
    if (!moving && ik_branch_suspect_) {
      std::vector<double> qc;
      bool out = false;
      const Eigen::Isometry3d cur = measuredPose(qc, out);
      const double pe = (target_.translation() - cur.translation()).norm();
      const double re = std::abs(Eigen::AngleAxisd(cur.linear().transpose() * target_.linear()).angle());
      if (pe < 3e-3 && re < 1e-2) {
        q_goal_ = qc;
        target_ = cur;
        target_ok_ = cur;
        have_target_ok_ = true;
        ik_branch_suspect_ = false;
      }
    }

    // ---- 2) 绝对目标积分（twist 在 base 系）----
    if (moving) {
      Eigen::Isometry3d cand = target_;
      const Eigen::Vector3d w = v.tail<3>();
      // 本拍转角 = |w|·dt_step。**必须乘 dt_step**：漏乘会让目标姿态每拍前进 |w| 弧度
      // （相当于 100 倍速），CLIK 追不上 → 每拍判"远离实测/换支"回退 → 实测超转 2~4 倍，
      // 且超转量与指令时长无关（实测指令 0.10 rad 走出 0.394 rad，减半到 0.05 仍是 0.353）。
      const double wnorm = w.norm();
      const double th = wnorm * dt_step;
      if (rotation_about_tip_) {
        // 绕末端点旋转：只改姿态，末端位置不变（R1 旋转 / 抬腕看向目标的语义）
        if (th > 1e-12) {
          const Eigen::Matrix3d rd = Eigen::AngleAxisd(th, w / wnorm).toRotationMatrix();
          cand.linear() = rd * target_.linear();
        }
        Eigen::Vector3d dp = v.head<3>() * dt_step;
        const double n = dp.norm();
        if (n > max_step_trans_ && n > 1e-9) dp *= max_step_trans_ / n;
        cand.translation() = target_.translation() + dp;
      } else {
        // 旧语义（rotation_about_tip:=false 时保留）：绕基座原点左乘，位置会画弧
        Eigen::Isometry3d d = Eigen::Isometry3d::Identity();
        if (th > 1e-12) d.linear() = Eigen::AngleAxisd(th, w / wnorm).toRotationMatrix();
        d.translation() = v.head<3>() * dt_step;
        cand = d * target_;
        Eigen::Vector3d dp = cand.translation() - target_.translation();
        const double n = dp.norm();
        if (n > max_step_trans_ && n > 1e-9) dp *= max_step_trans_ / n;
        cand.translation() = target_.translation() + dp;
      }
      // 工作空间夹紧
      for (int i = 0; i < 3; ++i) {
        cand.translation()[i] = clampd(cand.translation()[i], ws_min_[i], ws_max_[i]);
      }
      // z 锁：没有 z 指令时锁在当前高度；有 z 指令时跟随并更新锁点
      if (hold_z_ && std::abs(v[2]) < 1e-4) {
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
      double dmeas = 0.0;
      for (size_t i = 0; i < kDof; ++i) {
        dmax = std::max(dmax, std::abs(qsol[i] - q_goal_[i]));
        dmeas = std::max(dmeas, std::abs(qsol[i] - q_meas_[i]));
      }
      if (dmeas > max_ik_jump_meas_) {
        // 解离"实测关节"太远 → 姿态等价的另一分支（腕部伸直位形最典型）。
        // 这种解即便姿态正确，跟过去也会让工具多转几十度，必须拒绝。
        ++ik_far_rej_;
        ik_branch_suspect_ = true;
        if (have_target_ok_) target_ = target_ok_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "IK 解远离实测 %.3f rad > %.3f → 判为另一分支，保持当前目标（累计 %d）",
                             dmeas, max_ik_jump_meas_, ik_far_rej_);
      } else if (dmax < max_ik_jump_) {  // 正常：低通后接受
        if (goal_filter_tau_ > 1e-3) {
          const double a = std::min(1.0, dt_step / goal_filter_tau_);
          for (size_t i = 0; i < kDof; ++i) q_goal_[i] += a * (qsol[i] - q_goal_[i]);
        } else {
          q_goal_ = qsol;
        }
        ik_fail_streak_ = 0;
        target_ok_ = target_;       // 记录"可达"的目标，失败时回退到这里
        have_target_ok_ = true;
      } else if (ik_jump_clip_ && dmax < max_ik_jump_hard_) {
        // 解与种子相差较远（低通滞后，或腕部近奇异时两个近邻解）：**限步逼近**而不是整帧丢弃。
        // 丢弃会死锁：q_goal_ 冻住 → 下一拍 dmax 依旧超标 → 每拍都拒（实测 100 次/秒，
        // 指令 0.15 rad/s 只走出来 0.05 rad）。限步后每拍最多走 max_ik_jump_，连续且可控。
        const double s = max_ik_jump_ / dmax;
        for (size_t i = 0; i < kDof; ++i) q_goal_[i] += s * (qsol[i] - q_goal_[i]);
        ik_fail_streak_ = 0;
        ++ik_jump_clip_cnt_;
        ik_branch_suspect_ = true;
        target_ok_ = target_;
        have_target_ok_ = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "IK 限步跟进：dmax=%.3f rad > %.3f → 本拍走 %.3f（累计 %d）",
                             dmax, max_ik_jump_, max_ik_jump_, ik_jump_clip_cnt_);
      } else {
        ++ik_jump_reject_;
        ik_branch_suspect_ = true;
        if (have_target_ok_) target_ = target_ok_;   // 跳变过大（真换支）：目标回退，别继续积分
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "IK 换支被拒：dmax=%.3f rad ≥ %.3f（累计 %d）",
                             dmax, max_ik_jump_hard_, ik_jump_reject_);
      }
    } else {
      ++ik_fail_streak_;
      ++ik_fail_total_;
      // 目标不可达：回退到上次成功位姿，避免"目标跑飞、残差无限涨"（实测可达 700mm）
      if (have_target_ok_) target_ = target_ok_;
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
      const double a = (vj_filter_tau_ > 1e-3) ? std::min(1.0, dt_step / vj_filter_tau_) : 1.0;
      for (size_t i = 0; i < kDof; ++i) vj_f_[i] += a * (vj[i] - vj_f_[i]);
    }

    // ---- 5) 有状态速度积分器（前馈 + 位置闭环 + 加速度/加加速度限幅）----
    // 为什么不用"每拍重规划"：目标只领先 1 个周期时，时间最优解对目标的亚毫弧度抖动极其
    // 敏感（实测 IK 落点抖 0.5mrad → 输出 ±150% 速度摆振、甚至反向）。有状态积分器对同样的
    // 抖动只产生 Kp*0.5mrad 的响应，结构上不会自激；加速度另经阻尼加速度环（acc_gain）。
    const double a_step = max_jerk_ * dt_step;  // 每拍允许的加速度变化
    std::array<double, kDof> q_out{}, qd_out{}, qa_out{};
    for (size_t i = 0; i < kDof; ++i) {
      double v_des = vj_f_[i] + pos_gain_ * (q_goal_[i] - q_cmd_[i]);
      v_des = clampd(v_des, -max_vel_, max_vel_);
      const double a_des = clampd(acc_gain_ * (v_des - qd_cmd_[i]), -max_acc_, max_acc_);
      qa_cmd_[i] = clampd(a_des, qa_cmd_[i] - a_step, qa_cmd_[i] + a_step);
      qd_cmd_[i] = clampd(qd_cmd_[i] + qa_cmd_[i] * dt_step, -max_vel_, max_vel_);
      q_cmd_[i] += qd_cmd_[i] * dt_step;
      // F187：命令状态贴限位（不只是 IK 解），否则前馈会把命令推出量程。
      // 若实测本来就在余量带内（如 idle 的 L2=0.015 而限位 0.000 + 余量 0.020），
      // 不能把命令顶到余量线上——那等于常驻一个 5 mrad 的位置偏置（≈0.4 Nm 顶着发热），
      // 改为"最多退到实测处、不许再往限位里走"。
      const double lo_lim = std::min(joint_lo_[i] + cmd_limit_margin_, q_meas_[i]);
      const double hi_lim = std::max(joint_hi_[i] - cmd_limit_margin_, q_meas_[i]);
      if (q_cmd_[i] < lo_lim || q_cmd_[i] > hi_lim) {
        const bool lo = q_cmd_[i] < lo_lim;
        q_cmd_[i] = lo ? lo_lim : hi_lim;
        if (lo ? (qd_cmd_[i] < 0.0) : (qd_cmd_[i] > 0.0)) {
          qd_cmd_[i] = 0.0;
          qa_cmd_[i] = 0.0;
        }
        ++cmd_limit_clamp_cnt_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "命令到限位：%s q_cmd=%.3f（限位 %.3f..%.3f，余量 %.3f）"
                             " 累计 %lu 次；目标已贴限位，继续跟会顶死电机",
                             joint_names_[i].c_str(), q_cmd_[i],
                             joint_lo_[i], joint_hi_[i], cmd_limit_margin_,
                             static_cast<unsigned long>(cmd_limit_clamp_cnt_));
      }
      q_out[i] = q_cmd_[i];
      qd_out[i] = qd_cmd_[i];
      qa_out[i] = qa_cmd_[i];
    }

    // ---- 5b) 安全：命令跑得太前（被挡/跟不上）→ 命令状态回拉到实测并重锚 ----
    double track_err = 0.0;
    for (size_t i = 0; i < kDof; ++i) {
      track_err = std::max(track_err, std::abs(q_cmd_[i] - q_meas_[i]));
    }
    if (track_err > max_track_err_) {
      ++track_err_resets_;
      std::vector<double> qc;
      bool out = false;
      target_ = measuredPose(qc, out);
      for (size_t i = 0; i < kDof; ++i) {
        q_cmd_[i] = qc[i];
        qd_cmd_[i] = qd_meas_[i];
        qa_cmd_[i] = 0.0;
      }
      z_lock_ = target_.translation().z();
      target_ok_ = target_;
      have_target_ok_ = true;
      q_goal_ = qc;
      v_lim_.setZero();
      v_lp_.setZero();
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
      dbg_ << ',' << (ik_ok ? 1 : 0) << ",0\n";
    }

    // ---- 7) 低频诊断 ----
    const auto tnow = now();
    if ((tnow - last_log_t_).seconds() >= log_period_) {
      last_log_t_ = tnow;
      const Eigen::Vector3d err =
        target_.translation() - state_->getGlobalLinkTransform(tip_frame_).translation();
      RCLCPP_INFO(get_logger(),
                  "err=%.1fmm z=%.4f ik_fail=%d/%d ik_jump_rej=%d moving=%d |v|=%.3f",
                  err.norm() * 1e3, target_.translation().z(), ik_fail_streak_, ik_fail_total_,
                  ik_jump_reject_, moving ? 1 : 0, v.head<3>().norm());
    }
  }

  // params
  std::string group_name_, base_frame_, tip_frame_, twist_topic_, out_topic_, js_topic_;
  std::string mode_topic_;
  double mode_topic_timeout_{ 0.5 };
  double dt_{ 0.01 }, max_step_trans_{ 0.02 }, twist_timeout_{ 0.3 }, ik_timeout_{ 0.02 };
  double max_vel_{ 1.5 }, max_acc_{ 8.0 }, max_jerk_{ 200.0 }, max_ik_jump_{ 0.05 };
  bool ik_jump_clip_{ true };
  double max_ik_jump_hard_{ 0.60 };
  double max_ik_jump_meas_{ 0.20 };
  int ik_jump_clip_cnt_{ 0 };
  int ik_far_rej_{ 0 };
  bool use_clik_{ true };
  int clik_iters_{ 8 };
  double clik_damping_{ 0.02 }, clik_tol_pos_{ 1e-4 }, clik_tol_rot_{ 1e-4 }, clik_max_dq_{ 0.15 };
  double clik_tol_pos_loose_{ 0.005 }, clik_tol_rot_loose_{ 0.02 };
  std::vector<double> joint_lo_, joint_hi_;
  double goal_filter_tau_{ 0.06 };
  double twist_max_linear_{ 0.40 }, twist_max_angular_{ 1.2 };
  double twist_max_accel_{ 1.0 }, twist_max_ang_accel_{ 3.0 }, twist_lpf_tau_{ 0.06 };
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

  // ros
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_tw_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
  rclcpp::Subscription<a3_msgs::msg::ArmStatus>::SharedPtr sub_arm_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stale_;
  bool feedback_stale_{ false };
  std::string stale_topic_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reanchor_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  std::mutex mu_;
  bool have_state_{ false }, have_twist_{ false }, published_{ false }, was_in_servo_{ false };
  std::string mode_;
  std::string arm_state_;
  rclcpp::Time arm_state_t_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time mode_t_{ 0, 0, RCL_ROS_TIME };
  std::vector<double> q_meas_, qd_meas_, q_goal_;
  std::array<double, kDof> q_cmd_{}, qd_cmd_{}, qa_cmd_{};
  double settle_rate_{ 0.3 };            // [rad/s] 停发前把命令收到实测位姿的速率
  bool settle_active_{ false };
  // 归位目标必须是"进入时刻的快照"，不能是实时实测：用实时实测时，臂被重力拉低、
  // 命令就逐拍跟着往下走，差恒为 0（永不收敛）→ 无人脸时持续缓慢下沉（2026-10-25 实测）。
  std::array<double, kDof> settle_target_{};
  bool settle_done_{ false };            // 本次停发已归位完成（避免每拍重锚后再归位刷屏）
  double cmd_limit_margin_{ 0.02 };      // [rad] 命令与 URDF 限位的最小余量
  uint64_t cmd_limit_clamp_cnt_{ 0 };
  double max_track_err_{ 3.0 };  // [rad] 命令-实测偏差上限（只在真的被挡/失速时才重锚）
  double vj_filter_tau_{ 0.05 };
  double pos_gain_{ 4.0 };
  double acc_gain_{ 20.0 };
  std::array<double, kDof> vj_f_{};
  Eigen::Matrix<double, 6, 1> v_lim_{ Eigen::Matrix<double, 6, 1>::Zero() };  // 加速度限幅后
  Eigen::Matrix<double, 6, 1> v_lp_{ Eigen::Matrix<double, 6, 1>::Zero() };   // 低通输出
  int track_err_resets_{ 0 };
  geometry_msgs::msg::TwistStamped twist_;
  rclcpp::Time twist_t_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_log_t_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_step_t_{ 0, 0, RCL_ROS_TIME };
  Eigen::Isometry3d target_{ Eigen::Isometry3d::Identity() };
  // 最近一次 IK 解算成功的目标：IK 失败时回退到这里，避免目标继续积分跑飞（实测残差可达 700mm）
  Eigen::Isometry3d target_ok_{ Eigen::Isometry3d::Identity() };
  bool have_target_ok_{ false };
  bool rotation_about_tip_{ true };
  double z_lock_{ 0.0 };
  int ik_fail_streak_{ 0 }, ik_fail_total_{ 0 }, ik_jump_reject_{ 0 };
  // 本拍 IK 出现过"换支/远离实测/限步跟进"→ 下一拍且已到位时用实测重锚一次（见 1b）
  bool ik_branch_suspect_{ false };
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
