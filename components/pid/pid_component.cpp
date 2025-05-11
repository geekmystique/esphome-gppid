#include "pid_component.h"

#include "esphome/core/log.h"

namespace esphome {
namespace pid {

static const char *const TAG = "pid";

void PIDComponent::setup() {
    this->input_sensor_->add_on_state_callback([this](float state) {
        ESP_LOGD(TAG, "input sensor callback - got value %f", state);
        this->update_pid_(state);
    });
    if (this->target_sensor_ != nullptr) {
        this->target_sensor_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "target sensor callback - submitting value %f", state);
            this->target_value_ = state;
        });
    }
#ifdef USE_NUMBER
    if (this->target_number_ != nullptr) {
        this->target_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "target number callback - submitting value %f", state);
            if (std::isfinite(state)) this->target_value_ = state;
        });
    }
    if (this->kp_number_ != nullptr) {
        this->kp_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "kp callback - submitting value %f", state);
            if (std::isfinite(state)) this->set_kp(state);
        });
    }
    if (this->ki_number_ != nullptr) {
        this->ki_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "ki callback - submitting value %f", state);
            if (std::isfinite(state)) this->set_ki(state);
        });
    }
    if (this->kd_number_ != nullptr) {
        this->kd_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "kd callback - submitting value %f", state);
            if (std::isfinite(state)) this->set_kd(state);
        });
    }

#endif
}

void PIDComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "PID Controller", this);
    ESP_LOGCONFIG(TAG, "  Control Parameters:");
    ESP_LOGCONFIG(TAG, "    kp: %.5f, ki: %.5f, kd: %.5f",
                  controller_.kp_, controller_.ki_, controller_.kd_);

    if (controller_.threshold_low_ == 0 && controller_.threshold_high_ == 0) {
        ESP_LOGCONFIG(TAG, "  Deadband disabled.");
    } else {
        ESP_LOGCONFIG(TAG, "  Deadband Parameters:");
        ESP_LOGCONFIG(
            TAG,
            "    threshold: %0.5f to %0.5f, multipliers(kp: %.5f, ki: "
            "%.5f, kd: %.5f), output samples: %d",
            controller_.threshold_low_, controller_.threshold_high_,
            controller_.kp_multiplier_, controller_.ki_multiplier_,
            controller_.kd_multiplier_, controller_.deadband_output_samples_);
    }
}

void PIDComponent::update_pid_(float current_value) {
    if (std::isfinite(current_value) && std::isfinite(this->target_value_)) {
        float value = this->controller_.update(this->target_value_, current_value);
        ESP_LOGD(TAG, "update_pid: %f -> %f: %f", current_value, this->target_value_, value);
        ESP_LOGD(TAG, "write output value %f", value);
        this->output_value_ = value;
        this->pid_computed_callback_.call();
#ifdef USE_OUTPUT
        ESP_LOGD(TAG, "write output: tmp: %f", value);
        if (this->output_ != nullptr) {
            this->output_->set_level(value);
        }
#endif
    }
    else {
        ESP_LOGD(TAG, "nan");
    }
}

void PIDComponent::reset_integral_term() {
    this->controller_.reset_integral_term();
}

}  // namespace pid
}  // namespace esphome
