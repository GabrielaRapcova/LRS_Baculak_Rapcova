// Offline replay of a sensor event stream through the Localizer (no ROS).
//
// Input (CSV, one event per line, in arrival order):
//   imu,t,gyro_z
//   gps,t,x,y,z,var_x,var_y,var_z
//   vo,t,yaw,vbx,vby,vbz,var_vx,var_vy,var_vz
//   # comments and blank lines are ignored
//
// Output (CSV to stdout), one row per accepted IMU step after initialisation
// (every N-th with --every N):
//   t,x,y,z,vx,vy,vz,yaw,bias,sx,sy,sz,svx,svy,svz,syaw,sbias   (state order)
//
// Filter parameters can be overridden with key=value arguments, e.g.
//   ekf_replay events.csv accel_xy=0.8 gps_gate_chi2=0
// Logs and final statistics go to stderr.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "drone_localization/localizer.hpp"

using drone_localization::LocalizerConfig;
using drone_localization::Localizer;
using drone_localization::LogLevel;
using drone_localization::SensorStats;

namespace
{

bool apply_override(LocalizerConfig & c, const std::string & key, double v)
{
  std::map<std::string, double *> doubles = {
    {"accel_xy", &c.noise.accel_xy}, {"accel_z", &c.noise.accel_z},
    {"gyro_noise", &c.noise.gyro_noise}, {"gyro_rate", &c.noise.gyro_rate},
    {"bias_walk", &c.noise.bias_walk},
    {"init_yaw", &c.init_yaw}, {"init_yaw_sigma", &c.init_yaw_sigma},
    {"init_velocity_sigma", &c.init_velocity_sigma},
    {"init_bias_sigma", &c.init_bias_sigma},
    {"max_measurement_lag", &c.max_measurement_lag},
    {"gps_sigma_scale", &c.gps_sigma_scale}, {"gps_gate_chi2", &c.gps_gate_chi2},
    {"vo_velocity_sigma", &c.vo_velocity_sigma},
    {"vo_velocity_gate_chi2", &c.vo_velocity_gate_chi2},
    {"vo_yaw_sigma", &c.vo_yaw_sigma}, {"vo_yaw_period", &c.vo_yaw_period},
    {"vo_yaw_window", &c.vo_yaw_window}, {"vo_pose_yaw_sigma", &c.vo_pose_yaw_sigma},
    {"vo_yaw_rate_sigma", &c.vo_yaw_rate_sigma}, {"vo_yaw_gate_chi2", &c.vo_yaw_gate_chi2},
  };
  if (auto it = doubles.find(key); it != doubles.end()) {
    *it->second = v;
    return true;
  }
  if (key == "init_gps_samples") {c.init_gps_samples = static_cast<int>(v); return true;}
  if (key == "init_bias_from_gyro") {c.init_bias_from_gyro = v != 0.0; return true;}
  if (key == "gps_use_message_covariance") {c.gps_use_message_covariance = v != 0.0; return true;}
  if (key == "vo_velocity_enabled") {c.vo_velocity_enabled = v != 0.0; return true;}
  if (key == "vo_yaw_absolute") {c.vo_yaw_absolute = v != 0.0; return true;}
  if (key == "vo_yaw_relative") {c.vo_yaw_relative = v != 0.0; return true;}
  return false;
}

void print_stats(const char * name, const SensorStats & s)
{
  std::fprintf(stderr,
    "%-4s received=%zu accepted=%zu rejected=%zu stale=%zu late=%zu\n",
    name, s.received, s.accepted, s.rejected, s.stale, s.late);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string path;
  int every = 1;
  LocalizerConfig config;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--every" && i + 1 < argc) {
      every = std::max(1, std::atoi(argv[++i]));
    } else if (auto eq = arg.find('='); eq != std::string::npos) {
      if (!apply_override(config, arg.substr(0, eq), std::atof(arg.c_str() + eq + 1))) {
        std::fprintf(stderr, "unknown parameter '%s'\n", arg.c_str());
        return 2;
      }
    } else {
      path = arg;
    }
  }

  std::ifstream file;
  if (!path.empty()) {
    file.open(path);
    if (!file) {
      std::fprintf(stderr, "cannot open %s\n", path.c_str());
      return 1;
    }
  }
  std::istream & in = path.empty() ? std::cin : file;

  Localizer loc(config);
  loc.set_logger([](LogLevel level, const std::string & msg) {
      static const char * names[] = {"DEBUG", "INFO", "WARN", "ERROR"};
      std::fprintf(stderr, "[%s] %s\n", names[static_cast<int>(level)], msg.c_str());
    });

  std::printf("t,x,y,z,vx,vy,vz,yaw,bias,sx,sy,sz,svx,svy,svz,syaw,sbias\n");
  std::string line;
  std::size_t imu_rows = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::stringstream ss(line);
    std::string kind, field;
    std::getline(ss, kind, ',');
    std::vector<double> v;
    while (std::getline(ss, field, ',')) {v.push_back(std::atof(field.c_str()));}

    if (kind == "imu" && v.size() >= 2) {
      const std::size_t before = loc.imu_stats().accepted;
      loc.on_imu(v[0], v[1]);
      if (loc.imu_stats().accepted == before || (imu_rows++ % every) != 0) {continue;}
      const auto & x = loc.ekf().x();
      const auto sd = loc.ekf().P().diagonal().cwiseSqrt();
      std::printf("%.4f", loc.ekf().time());
      for (int k = 0; k < 8; ++k) {std::printf(",%.6f", x(k));}
      for (int k = 0; k < 8; ++k) {std::printf(",%.6f", sd(k));}
      std::printf("\n");
    } else if (kind == "gps" && v.size() >= 7) {
      Eigen::Matrix3d cov = Eigen::Vector3d(v[4], v[5], v[6]).asDiagonal();
      loc.on_gps(v[0], Eigen::Vector3d(v[1], v[2], v[3]), cov);
    } else if (kind == "vo" && v.size() >= 8) {
      loc.on_vo(v[0], v[1], Eigen::Vector3d(v[2], v[3], v[4]),
        Eigen::Vector3d(v[5], v[6], v[7]));
    }
  }

  print_stats("IMU", loc.imu_stats());
  print_stats("GPS", loc.gps_stats());
  print_stats("VOv", loc.vo_velocity_stats());
  print_stats("VOy", loc.vo_yaw_stats());
  return 0;
}
