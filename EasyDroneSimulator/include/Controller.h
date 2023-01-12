//
// Created by Baozhe Zhang on 2023/1/12.
//

#ifndef EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_CONTROLLER_H_
#define EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_CONTROLLER_H_

#include "Simulator.h"

namespace ez_drone_sim
{

// virtual class for quadrotor controller
class Controller
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using State_t = QuadrotorSystem::State_t;
  using Control_t = QuadrotorSystem::Control_t;
  using StateArray_t = QuadrotorSystem::StateArray_t;
  using InputArray_t = QuadrotorSystem::InputArray_t;
  Controller();
  virtual ~Controller() = 0;
  virtual Control_t operator()(double time, double duration,
      size_t sample_num,
      const StateArray_t &, const InputArray_t &) = 0;
};

class

} // namespace ez_drone_sim

#endif //EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_CONTROLLER_H_
