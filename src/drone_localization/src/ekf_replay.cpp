// Offline replay of a sensor event stream through the Localizer.
//
// Input (CSV, one event per line, in arrival order):
//   imu,t,gyro_z
//   gps,t,x,y,z,var_x,var_y,var_z
//   vo,t,yaw,vbx,vby,vbz,var_vx,var_vy,var_vz
//
// Output (CSV on stdout), one row per IMU prediction after initialisation:
//   t,x,y,z,vx,vy,vz,yaw,bias,sx,sy,sz,svx,svy,svz,syaw,sbias   (state, then sigmas)
//
// Parameters can be overridden as key=value, e.g. `ekf_replay events.csv accel_xy=0.2`.
// Logs and statistics go to stderr.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "drone_localization/localizer.hpp"

using drone_localization::Localizer;
using drone_localization::LocalizerConfig;

namespace
{

// Returns false for an unknown parameter name.
bool set_parameter(LocalizerConfig & c, const std::string & name, double value)
{
  const std::map<std::string, double *> parameters = {
    {"accel_xy", &c.noise.accel_xy},
    {"accel_z", &c.noise.accel_z},
    {"gyro_noise", &c.noise.gyro_noise},
    {"bias_walk", &c.noise.bias_walk},
    {"init_yaw_sigma", &c.init_yaw_sigma},
    {"init_velocity_sigma", &c.init_velocity_sigma},
    {"init_bias_sigma", &c.init_bias_sigma},
    {"init_bias_min_sigma", &c.init_bias_min_sigma},
    {"max_measurement_lag", &c.max_measurement_lag},
    {"gate_chi2_1dof", &c.gate_chi2_1dof},
    {"gate_chi2_3dof", &c.gate_chi2_3dof},
    {"vo_velocity_sigma", &c.vo_velocity_sigma},
    {"vo_yaw_window", &c.vo_yaw_window},
    {"vo_yaw_rate_sigma", &c.vo_yaw_rate_sigma},
    {"zaru_rate_sigma", &c.zaru_rate_sigma},
    {"zupt_sigma", &c.zupt_sigma},
  };
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return false;
  }
  *it->second = value;
  return true;
}

std::vector<double> parse_numbers(std::stringstream & line)
{
  std::vector<double> numbers;
  std::string field;
  while (std::getline(line, field, ',')) {
    numbers.push_back(std::atof(field.c_str()));
  }
  return numbers;
}

void print_row(const Localizer & localizer)
{
  const auto & x = localizer.ekf().x();
  const auto sigma = localizer.ekf().P().diagonal().cwiseSqrt();
  std::printf("%.4f", localizer.ekf().time());
  for (int i = 0; i < x.size(); ++i) {
    std::printf(",%.6f", x(i));
  }
  for (int i = 0; i < sigma.size(); ++i) {
    std::printf(",%.6f", sigma(i));
  }
  std::printf("\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string path;
  LocalizerConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto equals = arg.find('=');
    if (equals == std::string::npos) {
      path = arg;
    } else if (!set_parameter(config, arg.substr(0, equals), std::atof(arg.c_str() + equals + 1))) {
      std::fprintf(stderr, "unknown parameter '%s'\n", arg.c_str());
      return 2;
    }
  }

  std::ifstream file(path);
  if (!file) {
    std::fprintf(stderr, "usage: ekf_replay EVENTS.csv [key=value ...]\n");
    return 1;
  }

  Localizer localizer(config, rclcpp::get_logger("ekf_replay"));
  std::printf("t,x,y,z,vx,vy,vz,yaw,bias,sx,sy,sz,svx,svy,svz,syaw,sbias\n");

  std::string text;
  while (std::getline(file, text)) {
    std::stringstream line(text);
    std::string sensor;
    std::getline(line, sensor, ',');
    const std::vector<double> v = parse_numbers(line);

    if (sensor == "imu" && v.size() >= 2) {
      const double time_before = localizer.ekf().time();
      localizer.on_imu(v[0], v[1]);
      if (localizer.initialized() && localizer.ekf().time() > time_before) {
        print_row(localizer);
      }
    } else if (sensor == "gps" && v.size() >= 7) {
      const Eigen::Matrix3d covariance = Eigen::Vector3d(v[4], v[5], v[6]).asDiagonal();
      localizer.on_gps(v[0], Eigen::Vector3d(v[1], v[2], v[3]), covariance);
    } else if (sensor == "vo" && v.size() >= 8) {
      localizer.on_vo(v[0], v[1], Eigen::Vector3d(v[2], v[3], v[4]),
        Eigen::Vector3d(v[5], v[6], v[7]));
    }
  }

  const auto print_stats = [](const char * name, const drone_localization::SensorStats & s) {
      std::fprintf(stderr, "%-6s accepted=%d rejected=%d dropped=%d\n",
        name, s.accepted, s.rejected, s.dropped);
    };
  print_stats("GPS", localizer.gps_stats());
  print_stats("VO", localizer.vo_stats());
  print_stats("VOyaw", localizer.vo_yaw_stats());
  print_stats("ZARU", localizer.zaru_stats());
  print_stats("ZUPT", localizer.zupt_stats());
  return 0;
}
