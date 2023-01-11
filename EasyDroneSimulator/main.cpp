#include <iostream>

#include "QuadrotorSimulator.h"
#include "DisplayHandler.h"

using namespace ez_drone_sim;

// Reference: D. Mellinger and V. Kumar, “Minimum snap trajectory generation and control for quadrotors,” in 2011 IEEE International Conference on Robotics and Automation, Shanghai, China, May 2011, pp. 2520–2525. doi: 10.1109/ICRA.2011.5980409.
std::pair<QuadrotorSystem::StateArray_t , QuadrotorSystem::InputArray_t>
GenerateCircleTrajectory(size_t sample_num, double R, double omega, double duration, double start_z)
{
  QuadrotorSystem::StateArray_t state_trajectory;
  QuadrotorSystem::InputArray_t input_trajectory;
  std::pair<QuadrotorSystem::StateArray_t , QuadrotorSystem::InputArray_t> result;
  Eigen::Matrix3d temp_rotation_WB;
  Eigen::Quaterniond temp_q_WB;
  // thrust [m/s^2]
  double t = sqrt(pow(G, 2) + pow(omega * omega * R, 2));
  for (size_t i = 0; i < sample_num; i++) {
    Eigen::Vector3d x_B, y_B, z_B;
    x_B(0) = G / t * cos(omega * (i * duration / sample_num));
    x_B(1) = G / t * sin(omega * (i * duration / sample_num));
    x_B(2) = omega * omega * R / t;
    y_B(0) = -sin(omega * (i * duration / sample_num));
    y_B(1) = cos(omega * (i * duration / sample_num));
    y_B(2) = 0.0;
    z_B(0) = -omega * omega * R / t * cos(omega * (i * duration / sample_num));
    z_B(1) = -omega * omega * R / t * sin(omega * (i * duration / sample_num));
    z_B(2) = G / t;
    // Rotation matrix
    temp_rotation_WB <<
        x_B(0), y_B(0), z_B(0),
        x_B(1), y_B(1), z_B(1),
        x_B(2), y_B(2), z_B(2);
    temp_q_WB = Eigen::Quaterniond(temp_rotation_WB);
    temp_q_WB.normalize();
    Eigen::Vector3d a_dot, h_omega;
    a_dot(0) = omega * omega * omega * R * sin(i * duration / sample_num);
    a_dot(1) = -omega * omega * omega * R * cos(i * duration / sample_num);
    a_dot(2) = 0.0;
    h_omega = 1 / t * (a_dot - z_B.dot(a_dot) * z_B);
    QuadrotorSystem::State_t state = QuadrotorSystem::State_t::Zero();
    QuadrotorSystem::Control_t input = QuadrotorSystem::Control_t::Zero();
    state(0) = R * cos(omega * (i * duration / sample_num));
    state(1) = R * sin(omega * (i * duration / sample_num));
    state(2) = start_z;
    state(3) = -omega * R * sin(omega * (i * duration / sample_num));
    state(4) = omega * R * cos(omega * (i * duration / sample_num));
    state(5) = 0.0;
    state(6) = temp_q_WB.w();
    state(7) = temp_q_WB.x();
    state(8) = temp_q_WB.y();
    state(9) = temp_q_WB.z();
    input(0) = t;
    input(1) = -h_omega.dot(y_B);
    input(2) = h_omega.dot(x_B);
    input(3) = omega;
    state_trajectory.push_back(state);
    input_trajectory.push_back(input);
  }
  result.first = state_trajectory;
  result.second = input_trajectory;
  return result;
}

int main() {
  auto traj_pair = GenerateCircleTrajectory(1000, 2.0, 0.314, 20, 1.0);
  auto quad_system_ptr = std::make_shared<QuadrotorSystem>("Quadrotor");
  auto input_plan = [](double time, double duration, size_t sample_num,
      const QuadrotorSystem::StateArray_t &, const QuadrotorSystem::InputArray_t &input_trajectory)-> QuadrotorSystem::Control_t
  {
    size_t target_index = (size_t)(std::fmod(time, duration) / duration * (sample_num));
    return input_trajectory.at(target_index);
  };

  QuadrotorSystem::State_t x0 = QuadrotorSystem::State_t::Zero();
  x0 = traj_pair.first.at(0);
  double duration = 20.0 + 2.0;
  auto simulator = std::make_shared<QuadrotorSimulator>(0.005, 0.05, x0, quad_system_ptr);
  simulator->setReferenceInput(traj_pair.second);
  simulator->setInputPlan(input_plan);
  DisplayHandler display(simulator, "Simulator");
  simulator->simulate(duration);
  simulator->finish();
}
