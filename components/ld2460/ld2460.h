#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace ld2460 {

class LD2460Component;

class LD2460ReportingSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(LD2460Component *parent) { this->parent_ = parent; }
  void setup() override;

 protected:
  void write_state(bool state) override;

  LD2460Component *parent_{nullptr};
};

class LD2460Component : public Component, public uart::UARTDevice {
 public:
  static const uint8_t MAX_TARGETS = 5;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_summary_text_sensor(text_sensor::TextSensor *summary_text_sensor) {
    this->summary_text_sensor_ = summary_text_sensor;
  }
  void set_firmware_text_sensor(text_sensor::TextSensor *firmware_text_sensor) {
    this->firmware_text_sensor_ = firmware_text_sensor;
  }
  void set_installation_mode_text_sensor(text_sensor::TextSensor *installation_mode_text_sensor) {
    this->installation_mode_text_sensor_ = installation_mode_text_sensor;
  }
  void set_presence_binary_sensor(binary_sensor::BinarySensor *presence_binary_sensor) {
    this->presence_binary_sensor_ = presence_binary_sensor;
  }
  void set_target_count_sensor(sensor::Sensor *target_count_sensor) { this->target_count_sensor_ = target_count_sensor; }
  void set_reporting_switch(LD2460ReportingSwitch *reporting_switch) { this->reporting_switch_ = reporting_switch; }
  void set_reporting(bool enabled);
  void restore_reporting(bool enabled);
  void set_target_x_sensor(uint8_t index, sensor::Sensor *target_x_sensor);
  void set_target_y_sensor(uint8_t index, sensor::Sensor *target_y_sensor);
  void set_target_distance_sensor(uint8_t index, sensor::Sensor *target_distance_sensor);
  void set_target_angle_sensor(uint8_t index, sensor::Sensor *target_angle_sensor);
  void set_baud_scan(bool baud_scan) { this->baud_scan_ = baud_scan; }
  void set_flush_timeout(uint32_t flush_timeout_ms) { this->flush_timeout_ms_ = flush_timeout_ms; }
  void set_max_buffer_size(uint16_t max_buffer_size) { this->max_buffer_size_ = max_buffer_size; }
  void set_no_data_log_interval(uint32_t no_data_log_interval_ms) {
    this->no_data_log_interval_ms_ = no_data_log_interval_ms;
  }
  void set_publish_interval(uint32_t publish_interval_ms) { this->publish_interval_ms_ = publish_interval_ms; }
  void set_report_log_interval(uint32_t report_log_interval_ms) {
    this->report_log_interval_ms_ = report_log_interval_ms;
  }

 protected:
  struct TargetSensors {
    sensor::Sensor *x{nullptr};
    sensor::Sensor *y{nullptr};
    sensor::Sensor *distance{nullptr};
    sensor::Sensor *angle{nullptr};
  };

  struct Target {
    int16_t raw_x{0};
    int16_t raw_y{0};
    float x_m{0.0f};
    float y_m{0.0f};
    float distance_m{0.0f};
    float angle_deg{0.0f};
  };

  enum class StartupCommandState : uint8_t {
    IDLE,
    WAITING_FOR_DISABLE,
    WAITING_FOR_METADATA,
    WAITING_FOR_ENABLE,
    COMPLETE,
  };

  void send_startup_commands_();
  void send_enable_reporting_command_(bool enabled = true);
  void send_query_version_command_();
  void send_query_installation_mode_command_();
  void finish_metadata_queries_();
  void select_next_baud_rate_();
  void process_rx_buffer_();
  void process_report_frame_(const std::vector<uint8_t> &frame);
  void process_command_frame_(const std::vector<uint8_t> &frame);
  void publish_targets_(const Target *targets, uint8_t target_count, const std::string &summary);
  bool target_state_changed_(const Target *targets, uint8_t target_count) const;
  void remember_published_targets_(const Target *targets, uint8_t target_count);
  void flush_unparsed_buffer_();
  static std::string format_frame_(const std::vector<uint8_t> &bytes);
  static bool is_report_header_(const std::vector<uint8_t> &bytes);
  static bool is_command_header_(const std::vector<uint8_t> &bytes);
  static bool has_report_footer_(const std::vector<uint8_t> &bytes);
  static bool has_command_footer_(const std::vector<uint8_t> &bytes);
  static uint16_t read_u16_le_(const std::vector<uint8_t> &bytes, size_t index);
  static int16_t read_i16_le_(const std::vector<uint8_t> &bytes, size_t index);
  static const char *installation_mode_to_string_(uint8_t mode);

  text_sensor::TextSensor *summary_text_sensor_{nullptr};
  text_sensor::TextSensor *firmware_text_sensor_{nullptr};
  text_sensor::TextSensor *installation_mode_text_sensor_{nullptr};
  binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};
  sensor::Sensor *target_count_sensor_{nullptr};
  LD2460ReportingSwitch *reporting_switch_{nullptr};
  TargetSensors target_sensors_[MAX_TARGETS]{};
  Target last_published_targets_[MAX_TARGETS]{};
  std::vector<uint8_t> rx_buffer_{};
  std::vector<uint8_t> frame_buffer_{};
  std::string summary_buffer_{};
  uint32_t flush_timeout_ms_{100};
  uint16_t max_buffer_size_{48};
  uint32_t last_byte_ms_{0};
  uint32_t total_bytes_{0};
  uint32_t last_no_data_log_ms_{0};
  uint32_t last_command_ms_{0};
  uint32_t last_publish_ms_{0};
  uint32_t last_report_log_ms_{0};
  uint32_t publish_interval_ms_{500};
  uint32_t report_log_interval_ms_{1000};
  uint8_t last_published_target_count_{0};
  uint8_t baud_index_{0};
  bool baud_scan_{true};
  bool startup_commands_sent_{false};
  bool firmware_response_received_{false};
  bool installation_mode_response_received_{false};
  bool reporting_enabled_{true};
  bool reporting_restore_pending_{false};
  bool restore_reporting_after_metadata_{true};
  bool has_published_targets_{false};
  StartupCommandState startup_command_state_{StartupCommandState::IDLE};
  uint32_t no_data_log_interval_ms_{10000};
};

}  // namespace ld2460
}  // namespace esphome
