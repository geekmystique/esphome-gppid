#include "pid_controller.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace pid {

float PIDController::update(float setpoint, float process_value) {
  // e(t) ... error at timestamp t
  // r(t) ... setpoint
  // y(t) ... process value (sensor reading)
  // u(t) ... output value

  dt_ = calculate_relative_time_();

  // e(t) := r(t) - y(t)
  error_ = setpoint - process_value;

  calculate_proportional_term_();
  calculate_integral_term_();
  calculate_derivative_term_(setpoint);

  // u(t) := p(t) + i(t) + d(t)
  float output = proportional_term_ + integral_term_ + derivative_term_;

  // To prevent windup, if the output is outside its limits, we
  // recalculate the integral term to put the output at the limit.
  if (output < min_output_)
  {
      integral_term_ = min_output_ - proportional_term_ - derivative_term_;
      output = min_output_;
  }
  else if (output > max_output_)
  {
      integral_term_ = max_output_ - proportional_term_ - derivative_term_;
      output = max_output_;
  }
  
  return output;
}

bool PIDController::in_deadband() {
  // return (fabs(error) < deadband_threshold);
  float err = -error_;
  return (threshold_low_ < err && err < threshold_high_);
}

void PIDController::calculate_proportional_term_() {
  // p(t) := K_p * e(t)
  proportional_term_ = kp_ * error_;

  // set dead-zone to -X to +X
  if (in_deadband()) {
    // shallow the proportional_term in the deadband by the pdm
    proportional_term_ *= kp_multiplier_;

  } else {
    // pdm_offset prevents a jump when leaving the deadband
    float threshold = (error_ < 0) ? threshold_high_ : threshold_low_;
    float pdm_offset = (threshold - (kp_multiplier_ * threshold)) * kp_;
    proportional_term_ += pdm_offset;
  }
}

void PIDController::calculate_integral_term_() {
  // i(t) := K_i * \int_{0}^{t} e(t) dt
  float new_integral = error_ * dt_ * ki_;

  if (in_deadband()) {
      // shallow the integral when in the deadband
      integral_term_ += new_integral * ki_multiplier_;
  } else {
    integral_term_ += new_integral;
  }
}

void PIDController::calculate_derivative_term_(float setpoint) {
  // derivative_term_
  // d(t) := K_d * de(t)/dt
  float derivative = 0.0f;
  if (dt_ != 0.0f) {
    // remove changes to setpoint from error
    if (!std::isnan(previous_setpoint_) && previous_setpoint_ != setpoint)
      previous_error_ -= previous_setpoint_ - setpoint;
    derivative = (error_ - previous_error_) / dt_;
  }
  previous_error_ = error_;
  previous_setpoint_ = setpoint;

  derivative_term_ = kd_ * derivative;

  if (in_deadband()) {
    // shallow the derivative when in the deadband
    derivative_term_ *= kd_multiplier_;
  }
}


float PIDController::calculate_relative_time_() {
  uint32_t now = millis();
  uint32_t dt = now - this->last_time_;
  if (last_time_ == 0) {
    last_time_ = now;
    return 0.0f;
  }
  last_time_ = now;
  return dt / 1000.0f;
}

}  // namespace pid
}  // namespace esphome
