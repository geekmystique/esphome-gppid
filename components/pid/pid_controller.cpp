#include "pid_controller.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace pid {

float PIDController::update(float setpoint, float process_value,
                            float feedforward) {
    // ff(t) .. feed-forward control input
    // e(t) ... error at timestamp t
    // r(t) ... setpoint
    // y(t) ... process value (sensor reading)
    // u(t) ... output value

    dt_ = calculate_relative_time_();
    
    if (enable_) {
        // Only update controller values if enabled

        // e(t) := r(t) - y(t)
        error_ = setpoint - process_value;

        calculate_proportional_term_();
        calculate_derivative_term_(setpoint);
    }

    // FF value, if present, is passed through regardless of enable state.
    float const valid_ff = std::isnan(feedforward) ? 0 : feedforward;

    // First calculate tentative output before adding the integral.
    // u(t) := ff(t) + p(t) + i(t) + d(t)
    float const tmp_output =
        valid_ff + proportional_term_ + integral_term_ + derivative_term_;

    // Update integral term. To prevent windup, if the output is
    // outside the limits, we do not add the integral term if doing so
    // would bring the output more outside the limits. We do let it
    // evolve inward, though.
    if (enable_ &&
        !(((tmp_output < min_output_) && (error_*ki_ < 0)) ||
          ((tmp_output > max_output_) && (error_*ki_ > 0)))) {
        calculate_integral_term_();
    }

    // Recalculate output after updating integral term
    float const output = valid_ff + proportional_term_ + integral_term_ + derivative_term_;

    return std::min(std::max(output, min_output_), max_output_);
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
  float const new_integral = error_ * dt_ * ki_;

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
