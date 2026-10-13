// F167.3: 实时在线轨迹生成（OTG）数值演示（仿真-only，不接生产链路）。
// 复用栈内已链接的 Ruckig（jerk-limited OTG），生成三组场景的 (t, q, v, a) CSV，
// 供 scripts/a3_test/f167_otg_report.py 出 SVG + 指标：
//   1) sync      : 6 轴异距目标，Ruckig Time 同步（全轴同时到达）
//   2) async     : 同上但 Synchronization::None（各轴独立，作"无同步"对照）
//   3) retarget  : 运动中 t=0.5s 切换目标，从当前状态续算（不停车、速度连续）
//   4) brake     : 运动中触发急停，jerk-limited 制动到零速
// 用法： otg_demo [out_dir]   （默认 /tmp）
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include <ruckig/ruckig.hpp>

namespace {
constexpr size_t DOF = 6;
constexpr double kDt = 0.005;             // 200 Hz
constexpr double kMaxVel = 1.5;           // rad/s
constexpr double kMaxAcc = 6.0;           // rad/s^2
constexpr double kMaxJerk = 30.0;         // rad/s^3

using Ruckig = ruckig::Ruckig<DOF>;
using Input = ruckig::InputParameter<DOF>;
using Output = ruckig::OutputParameter<DOF>;

void set_uniform(Input& in) {
  for (size_t i = 0; i < DOF; ++i) {
    in.max_velocity[i] = kMaxVel;
    in.max_acceleration[i] = kMaxAcc;
    in.max_jerk[i] = kMaxJerk;
  }
}

void write_row(std::ofstream& f, double t, const Output& o) {
  f << t;
  for (size_t i = 0; i < DOF; ++i) f << "," << o.new_position[i];
  for (size_t i = 0; i < DOF; ++i) f << "," << o.new_velocity[i];
  for (size_t i = 0; i < DOF; ++i) f << "," << o.new_acceleration[i];
  f << "\n";
}

void open_csv(std::ofstream& f, const std::string& path) {
  f.open(path);
  f << "#t";
  for (size_t i = 0; i < DOF; ++i) f << ",q" << i;
  for (size_t i = 0; i < DOF; ++i) f << ",v" << i;
  for (size_t i = 0; i < DOF; ++i) f << ",a" << i;
  f << "\n";
}

// 场景 1/2：多轴同步 vs 无同步
void run_sync(const std::string& path, ruckig::Synchronization sync) {
  Ruckig otg(kDt);
  Input in;
  Output out;
  set_uniform(in);
  const std::array<double, DOF> tgt = {1.0, 0.5, -0.8, 0.3, -0.2, 0.6};
  for (size_t i = 0; i < DOF; ++i) {
    in.current_position[i] = 0.0;
    in.current_velocity[i] = 0.0;
    in.current_acceleration[i] = 0.0;
    in.target_position[i] = tgt[i];
    in.target_velocity[i] = 0.0;
    in.target_acceleration[i] = 0.0;
  }
  in.synchronization = sync;
  std::ofstream f;
  open_csv(f, path);
  ruckig::Result res;
  do {
    res = otg.update(in, out);
    write_row(f, out.time, out);
    out.pass_to_input(in);
  } while (res == ruckig::Result::Working);
}

// 场景 3：运动中改目标（不停车）
void run_retarget(const std::string& path) {
  Ruckig otg(kDt);
  Input in;
  Output out;
  set_uniform(in);
  for (size_t i = 0; i < DOF; ++i) {
    in.current_position[i] = 0.0;
    in.current_velocity[i] = 0.0;
    in.current_acceleration[i] = 0.0;
    in.target_position[i] = (i < 3) ? 1.2 : 0.9;  // 目标 A
    in.target_velocity[i] = 0.0;
    in.target_acceleration[i] = 0.0;
  }
  std::ofstream f;
  open_csv(f, path);
  ruckig::Result res;
  bool retargeted = false;
  do {
    res = otg.update(in, out);
    write_row(f, out.time, out);
    out.pass_to_input(in);  // in.current_* = out.new_*（先接管当前状态）
    if (!retargeted && out.time >= 0.5) {
      // 切换目标 B（用当前状态续算，Ruckig 检测输入变化自动重规划，不停车）
      for (size_t i = 0; i < DOF; ++i) {
        in.target_position[i] = (i < 3) ? 2.0 : 0.2;
        in.target_velocity[i] = 0.0;
        in.target_acceleration[i] = 0.0;
      }
      retargeted = true;
    }
  } while (res == ruckig::Result::Working);
}

// 场景 4：急停/制动
void run_brake(const std::string& path) {
  Ruckig otg(kDt);
  Input in;
  Output out;
  set_uniform(in);
  for (size_t i = 0; i < DOF; ++i) {
    in.current_position[i] = 0.0;
    in.current_velocity[i] = 0.0;
    in.current_acceleration[i] = 0.0;
    in.target_position[i] = 2.0;  // 远端目标（先加速跑）
    in.target_velocity[i] = 0.0;
    in.target_acceleration[i] = 0.0;
  }
  std::ofstream f;
  open_csv(f, path);
  ruckig::Result res;
  bool braked = false;
  do {
    res = otg.update(in, out);
    write_row(f, out.time, out);
    out.pass_to_input(in);  // 先接管当前状态
    if (!braked && out.time >= 0.3) {
      // 触发急停：目标=当前位置、零速（jerk-limited 制动）
      for (size_t i = 0; i < DOF; ++i) {
        in.target_position[i] = out.new_position[i];
        in.target_velocity[i] = 0.0;
        in.target_acceleration[i] = 0.0;
      }
      braked = true;
    }
  } while (res == ruckig::Result::Working);
}
}  // namespace

int main(int argc, char** argv) {
  const std::string dir = (argc > 1) ? argv[1] : "/tmp";
  run_sync(dir + "/otg_sync.csv", ruckig::Synchronization::Time);
  run_sync(dir + "/otg_async.csv", ruckig::Synchronization::None);
  run_retarget(dir + "/otg_retarget.csv");
  run_brake(dir + "/otg_brake.csv");
  std::cout << "otg_demo -> " << dir << "/otg_{sync,async,retarget,brake}.csv\n";
  return 0;
}
