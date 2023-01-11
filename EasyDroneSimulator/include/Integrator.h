//
// Created by Baozhe Zhang on 2023/1/10.
//

#ifndef EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_INTEGRATOR_H_
#define EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_INTEGRATOR_H_

#include <vector>
#include <iostream>
#include <cmath>
#include <memory>

#include <boost/numeric/odeint.hpp>

#include <System.h>

namespace ez_drone_sim
{

template <size_t STATE_DIM, size_t CONTROL_DIM>
class Integrator
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using State_t = typename System<STATE_DIM, CONTROL_DIM>::State_t;
  using Control_t = typename System<STATE_DIM, CONTROL_DIM>::Control_t;
  using StateArray_t = typename System<STATE_DIM, CONTROL_DIM>::StateArray_t ;
  using InputArray_t = typename System<STATE_DIM, CONTROL_DIM>::InputArray_t;
  using TimeArray_t = typename System<STATE_DIM, CONTROL_DIM>::TimeArray_t ;

  struct Observer
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // logging array
    StateArray_t &state_array_;
    TimeArray_t &time_array_;

    Observer(StateArray_t &state_array, TimeArray_t &time_array) :
        state_array_(state_array),
        time_array_(time_array)
    {
    }

    void reset()
    {
      if (state_array_.empty() && time_array_.empty())
        return;
      state_array_.clear();
      time_array_.clear();
    }

    // observer logging function
    void operator()(const State_t &x, double t)
    {
      state_array_.push_back(x);
      time_array_.push_back(t);
    }
  };

  // constructor
  Integrator(const std::shared_ptr<System<STATE_DIM, CONTROL_DIM>> &system_ptr) :
  system_ptr_(system_ptr),
  observer(state_array_, time_array_)
  {
    system_function_ =
        [this](const State_t &x, State_t &dxdt, double t)
        {
          system_ptr_->updateDynamics(x, t, dxdt);
        };
    rk4_stepper_ptr_ = std::make_shared<boost::numeric::odeint::runge_kutta4<State_t, double, State_t, double, boost::numeric::odeint::vector_space_algebra>>();
  }

  StateArray_t getStateArray() { return state_array_; }
  TimeArray_t getTimeArray() { return time_array_; }


  // update in each iteration
  void integrate_n_steps(State_t &state,
                         double start_time, size_t num_steps, double dt)
  {
    observer.reset();
    boost::numeric::odeint::integrate_n_steps(*rk4_stepper_ptr_, system_function_, state, start_time, dt, num_steps, observer);
  }


 private:

  // system instance
  std::shared_ptr<System<STATE_DIM, CONTROL_DIM>> system_ptr_;

  // function to odeint to integrate
  std::function<void(const State_t &, State_t &, double)> system_function_;

  // stepper instance
  std::shared_ptr<boost::numeric::odeint::runge_kutta4<
      State_t, double, State_t, double, boost::numeric::odeint::vector_space_algebra>> rk4_stepper_ptr_;

  // logging array
  StateArray_t state_array_;
  TimeArray_t time_array_;
  Observer observer;
};


} // namespace ez_drone_sim

#endif //EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_INTEGRATOR_H_
