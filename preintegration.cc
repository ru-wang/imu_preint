#include <cassert>

#include <algorithm>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <experimental/filesystem>

#include <Eigen/Eigen>

#include <glm/glm.hpp>

#include "bfvio/preintegration.h"
#include "bfvio/slam_trajectory_drawer.h"

using namespace Eigen;
using namespace std;
namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[]) {
  assert(argc > 1);
  string path = argv[1];

  /************************************************************
   * load files
   ************************************************************/

  regex acce_rex(".*acce_([0-9]+)-([0-9]+)-([0-9]+) ([0-9]+):([0-9]+):([0-9]+)\\.txt");
  regex gyro_rex(".*gyro_([0-9]+)-([0-9]+)-([0-9]+) ([0-9]+):([0-9]+):([0-9]+)\\.txt");
  smatch match;

  list<pair<string, string>> files;
  for (auto& p: fs::directory_iterator(path)) {
    string filename = p.path().string();
    if (!regex_match(filename, match, acce_rex))
      continue;
    string fid = string(match[1]) + string(match[2]) + string(match[3]) +
                 string(match[4]) + string(match[5]) + string(match[6]);
    auto it = files.begin();
    for (; it != files.end() && it->first < fid; ++it);
    files.insert(it, make_pair(fid, filename));
  }

  cout << "\nReading IMU files ...\n";

  vector<pair<unsigned long long, Vector3d>> acce_readings;
  for (auto& file : files) {
    cout << "  [" << file.first << "] \"" << file.second << "\"\n";
    ifstream ifs(file.second);
    string line;
    while (getline(ifs, line)) {
      istringstream iss(line);
      auto& acce = acce_readings.emplace_back();
      if (!(iss >> acce.first))
        break;
      long i;
      for (i = 0; i < acce.second.size() && iss >> acce.second[i]; ++i);
      if (i < acce_readings.front().second.size()) {
        acce_readings.pop_back();
        break;
      }
    }
    ifs.close();
  }

  files.clear();
  for (auto& p: fs::directory_iterator(path)) {
    string filename = p.path().string();
    if (!regex_match(filename, match, gyro_rex))
      continue;
    string fid = string(match[1]) + string(match[2]) + string(match[3]) +
                 string(match[4]) + string(match[5]) + string(match[6]);
    auto it = files.begin();
    for (; it != files.end() && it->first < fid; ++it);
    files.insert(it, make_pair(fid, filename));
  }

  vector<pair<unsigned long long, Vector3d>> gyro_readings;
  for (auto& file : files) {
    cout << "  [" << file.first << "] \"" << file.second << "\"\n";
    ifstream ifs(file.second);
    string line;
    while (getline(ifs, line)) {
      istringstream iss(line);
      auto& gyro = gyro_readings.emplace_back();
      if (!(iss >> gyro.first))
        break;
      long i;
      for (i = 0; i < gyro.second.size() && iss >> gyro.second[i]; ++i);
      if (i < gyro_readings.front().second.size()) {
        gyro_readings.pop_back();
        break;
      }
    }
    ifs.close();
  }

  /************************************************************
   * merge IMU readings
   ************************************************************/

  vector<IMU> imu_readings;
  Matrix<double, 9, 1> imu_reading = Matrix<double, 9, 1>::Zero();
  for (size_t i = 0, j = 0; i < gyro_readings.size() && j < acce_readings.size();) {
    unsigned long long t;
    if (acce_readings[i].first < gyro_readings[j].first) {
    } else if (acce_readings[i].first > gyro_readings[j].first) {
    } else if (acce_readings[i].first == gyro_readings[j].first) {
      imu_reading.block<3, 1>(0, 0) = gyro_readings[i].second;
      imu_reading.block<3, 1>(3, 0) = acce_readings[j].second;
      ++i, ++j;
    }
    imu_readings.emplace_back(std::move(imu_reading), t);
  }

  cout << "\nTotal IMU frames: " << imu_readings.size() << "\n"
       << "\nProcessing IMU readings ...\n";

  IMUState first_state;
  Vector3d bg = Vector3d::Zero(), ba = Vector3d::Zero(), g = Vector3d::Zero();
  for (size_t i = 0; i < 2000; ++i) {
    ba += imu_readings[i].a;
    bg += imu_readings[i].w;
  }
  ba /= 2000;
  bg /= 2000;
  first_state.bg = bg, first_state.ba = ba;

  vector<IMUState> states {first_state};
  Preintegrator preintegrator;

  imu_readings.erase(imu_readings.begin(), imu_readings.begin() + 2000);
  IMUState I_ij;
  IMUJacobi J_ij;
  IMUCov S_ij;
  BiasCov Q_ij;
  double t_ij;
  std::tie(I_ij, J_ij, S_ij, Q_ij, t_ij) = preintegrator.Integrate(imu_readings, bg, ba);

  cout << t_ij << " : " << I_ij.p << "\n";

  return 0;

  IMU first_imu(imu_readings[2000]);
  vector<IMU> imus;
  const size_t step = 10;
  for (size_t i = 2001; i < imu_readings.size(); i += step) {
    imus.clear();
    imus.push_back(first_imu);

    if (i + step > imu_readings.size()) {
      std::copy(imu_readings.cbegin() + i,
                imu_readings.cend(),
                std::back_inserter(imus));
    } else {
      std::copy(imu_readings.cbegin() + i,
                imu_readings.cbegin() + i + step,
                std::back_inserter(imus));
    }

    IMUState I_ij;
    IMUJacobi J_ij;
    IMUCov S_ij;
    BiasCov Q_ij;
    double t_ij;
    std::tie(I_ij, J_ij, S_ij, Q_ij, t_ij) = preintegrator.Integrate(imus, bg, ba);

    const Matrix3d& R_i = states.back().R;
    const Vector3d& p_i = states.back().p;
    const Vector3d& v_i = states.back().v;

    Matrix3d R_j = R_i * I_ij.R;
    Vector3d p_j = p_i + R_i * I_ij.p + v_i * t_ij + 0.5 * g * t_ij * t_ij;
    Vector3d v_j = v_i + R_i * I_ij.v + g * t_ij;

    states.emplace_back(R_j, p_j, v_j, bg, ba);

    first_imu = imus.back();
  }

  vector<glm::vec3> loc_pre_int;
  vector<glm::vec4> quat_pre_int;
  for (auto state : states) {
    loc_pre_int.emplace_back(state.p[0], state.p[1], state.p[2]);
    JPLQuaternion quat = RotMatToJPLQuat(state.R);
    quat_pre_int.emplace_back(quat.x(), quat.y(), quat.z(), quat.w());
  }

  SLAMTrajectoryDrawer::ReadTrajectoryFrom(loc_pre_int, quat_pre_int);
  SLAMTrajectoryDrawer::SetupGLUT(argc, argv);
  SLAMTrajectoryDrawer::SetupGLEW();
  SLAMTrajectoryDrawer::SetupGLSL();
  SLAMTrajectoryDrawer::StartDrawing();
  SLAMTrajectoryDrawer::FreeTrajectory();

  return 0;
}
