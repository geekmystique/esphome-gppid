#pragma once
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome {
namespace pid {

struct PIDController {
  float update(float setpoint, float process_value);

  void reset_integral_term() { integral_term_ = 0; }
  void set_starting_integral_term(float in) { integral_term_ = in; }

  bool in_deadband();

  friend class PIDComponent;

 private:
  /// Proportional gain K_p.
  float kp_ = 0;
  /// Integral gain K_i.
  float ki_ = 0;
  /// Differential gain K_d.
  float kd_ = 0;

  float threshold_low_ = 0.0f;
  float threshold_high_ = 0.0f;
  float kp_multiplier_ = 0.0f;
  float ki_multiplier_ = 0.0f;
  float kd_multiplier_ = 0.0f;
  int deadband_output_samples_ = 1;

    float min_output_ = NAN;
    float max_output_ = NAN;
    
  // Store computed values in struct so that values can be monitored through sensors
  float error_;
  float dt_;
  float proportional_term_;
  float integral_term_ = 0;
  float derivative_term_;

    void calculate_proportional_term_();
  void calculate_integral_term_();
  void calculate_derivative_term_(float setpoint);
  float calculate_relative_time_();

  /// Error from previous update used for derivative term
  float previous_error_ = 0;
  float previous_setpoint_ = NAN;
  uint32_t last_time_ = 0;

};  // Struct PID Controller
}  // namespace pid
}  // namespace esphome
