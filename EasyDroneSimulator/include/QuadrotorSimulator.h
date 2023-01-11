//
// Created by Baozhe Zhang on 2023/1/10.
//

#ifndef EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_QUADROTORSIMULATOR_H_
#define EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_QUADROTORSIMULATOR_H_

#include "Simulator.h"



namespace ez_drone_sim {

class QuadrotorSimulator : public NumericalSimulator<QuadrotorSystem> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base_t = NumericalSimulator<QuadrotorSystem>;


  // constructor
  QuadrotorSimulator(double system_dt,
                     double control_dt,
                     const State_t &x0,
                     std::shared_ptr<QuadrotorSystem> system_ptr,
                     bool verbose = true) :
      Base_t(system_dt, control_dt, x0, system_ptr, verbose)
  {
  }

  State_t getState()
  {
    return x_;
  }

  bool isFinished()
  {
    return !system_thread_.joinable() && !control_thread_.joinable();
  }

  // FIXME: if this virtual destructor is not specified or
  // finish() is not called (i.e., the threads are joined because of the destructor)
  // then the following overriden functions are not properly dispatched
  virtual ~QuadrotorSimulator() { finish(); }

  virtual void finishSystemIteration(double sim_time) override {
    printf("[System Iteration]\t t: %.3f \tx: %.3f \ty: %.3f \tz: %.3f\n"
           "\t r: %.3f \t theta: %.3f\n", sim_time, x_(0), x_(1), x_(2),
           sqrt(x_(0) * x_(0) + x_(1) * x_(1)), 180 / 3.14 * atan2(x_(1), x_(0)));
  }

  virtual void prepareControllerIteration(double sim_time) override {
    //
  }

  virtual void finishControllerIteration(double sim_time) override {
    state_mtx_.lock();
    State_t x_temp = x_;
    state_mtx_.unlock();

    system_ptr_->setControlAction(input_plan_(
        sim_time,
        trajectory_duration_,
        trajectory_sample_num_,
        reference_trajectory_,
        reference_input_
        ));
  }

  void setReferenceTrajectory(const QuadrotorSystem::StateArray_t &ref_traj)
  {
    reference_trajectory_ = ref_traj;
  }

  void setReferenceInput(const QuadrotorSystem::InputArray_t &ref_input)
  {
    reference_input_ = ref_input;
    system_ptr_->setControlAction(Eigen::Vector4d(G, 0.0, 0.0, 0.0));
  }

  void setInputPlan(const std::function<Control_t(double, double, size_t, const QuadrotorSystem::StateArray_t &, const QuadrotorSystem::InputArray_t &)> &plan)
  {
    input_plan_ = plan;
  }

 private:
  QuadrotorSystem::StateArray_t reference_trajectory_;
  QuadrotorSystem::InputArray_t reference_input_;
  size_t trajectory_sample_num_;
  double trajectory_duration_;
  std::function<State_t(double, double, size_t, const QuadrotorSystem::StateArray_t &, const QuadrotorSystem::InputArray_t &)>
      state_plan_;
  std::function<Control_t(double, double, size_t, const QuadrotorSystem::StateArray_t &, const QuadrotorSystem::InputArray_t &)>
      input_plan_;

};
} // namespace ez_drone_sim

#endif //EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_QUADROTORSIMULATOR_H_
