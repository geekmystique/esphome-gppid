#include "pid_component.h"

#include "esphome/core/log.h"

namespace esphome {
namespace pid {

static const char *const TAG = "pid";

void PIDComponent::setup() {
    this->input_sensor_->add_on_state_callback([this](float state) {
        ESP_LOGD(TAG, "input sensor callback - got value %f", state);
        this->input_value_ = state;
        this->update_pid_(this->input_value_, this->feedforward_value_);
    });
    if (this->target_sensor_ != nullptr) {
        this->target_sensor_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "target sensor callback - submitting value %f", state);
            this->target_value_ = state;
        });
    }
    if (this->enable_switch_ != nullptr) {

        this->enable_switch_->add_on_state_callback([this](
            bool state) {
            ESP_LOGD(TAG, "enable switch callback - submitting value %d", state);
            this->set_enable(state);
        });
    }
#ifdef USE_NUMBER
    if (this->ff_input_number_ != nullptr) {
        this->ff_input_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "ff input number callback - submitting value %f", state);
            if (std::isfinite(state)) {
                this->feedforward_value_ = state;
                this->update_pid_(this->input_value_, this->feedforward_value_);
            }
        });
    }
    if (this->target_number_ != nullptr) {
        this->target_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "target number callback - submitting value %f", state);
            if (std::isfinite(state)) this->target_value_ = state;
        });
        if (this->target_number_->has_state())
        {
            float const state = this->target_number_->state;
            ESP_LOGD(TAG, "updating target number - submitting value %f", state);
            if (std::isfinite(state)) this->target_value_ = state;
        }            
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
    if (this->min_output_number_ != nullptr) {
        this->min_output_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "min output callback - submitting value %f", state);
            if (std::isfinite(state)) this->set_min_output(state);
        });
        if (this->min_output_number_->has_state()) {
            float const state = this->min_output_number_->state;
            ESP_LOGD(TAG, "updating min output - submitting value %f", state);
            if (std::isfinite(state)) this->set_min_output(state);
        }
    }


    if (this->max_output_number_ != nullptr) {
        this->max_output_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "max output callback - submitting value %f", state);
            if (std::isfinite(state)) this->set_max_output(state);
        });
    if (this->max_output_number_->has_state()) {
            float const state = this->max_output_number_->state;
            ESP_LOGD(TAG, "updating max output - submitting value %f", state);
            if (std::isfinite(state)) this->set_max_output(state);
        }
    }

#endif
}

void PIDComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "PID Controller", this);
    ESP_LOGCONFIG(TAG, "  Control Parameters:");
    ESP_LOGCONFIG(TAG, "    kp: %.5f, ki: %.5f, kd: %.5
