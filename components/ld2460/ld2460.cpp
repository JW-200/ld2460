#include "ld2460.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cctype>
#include <cinttypes>
#include <cmath>
#include <cstdio>

namespace esphome {
namespace ld2460 {

static const char *const TAG = "ld2460";
static const uint32_t BAUD_RATES[] = {115200, 9600, 19200, 38400, 57600, 230400, 256000, 460800};
static const float RAD_TO_DEG = 57.2957795131f;
// Do not let a continuously busy UART monopolize ESPHome's main loop.  The
// remaining bytes stay in the UART driver's ring buffer for the next loop.
static const size_t MAX_BYTES_PER_LOOP = 128;

void LD2460ReportingSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_reporting(state);
}

void LD2460ReportingSwitch::setup() {
  const auto restored_state = this->get_initial_state_with_restore_mode();
  if (restored_state.has_value() && this->parent_ != nullptr)
    this->parent_->restore_reporting(*restored_state);
}

void LD2460ConfigNumber::control(float value) {
  if (this->parent_ != nullptr)
    this->parent_->set_config_value(this->field_, value);
}

void LD2460InstallationModeSelect::control(const std::string &value) {
  if (this->parent_ != nullptr)
    this->parent_->set_installation_mode(value);
}

void LD2460Component::setup() {
  this->rx_buffer_.reserve(this->max_buffer_size_);
  this->frame_buffer_.reserve(this->max_buffer_size_);
  // Reuse storage for continuous report summaries instead of allocating and
  // freeing heap memory for every frame.
  this->summary_buffer_.reserve(384);
}

void LD2460Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2460 raw UART reader:");
  ESP_LOGCONFIG(TAG, "  Flush timeout: %" PRIu32 " ms", this->flush_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Max buffer size: %u byte(s)", this->max_buffer_size_);
  ESP_LOGCONFIG(TAG, "  Baud scan: %s", YESNO(this->baud_scan_));
  ESP_LOGCONFIG(TAG, "  No-data log interval: %" PRIu32 " ms", this->no_data_log_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Publish interval: %" PRIu32 " ms", this->publish_interval_ms_);
  this->check_uart_settings(115200, 1, uart::UART_CONFIG_PARITY_NONE, 8);
  LOG_TEXT_SENSOR("  ", "Summary", this->summary_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Firmware", this->firmware_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Installation Mode", this->installation_mode_text_sensor_);
  LOG_BINARY_SENSOR("  ", "Presence", this->presence_binary_sensor_);
  LOG_SENSOR("  ", "Target Count", this->target_count_sensor_);
  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    const auto target = this->target_sensors_[i];
    char prefix[24];
    std::snprintf(prefix, sizeof(prefix), "  Target %u ", static_cast<unsigned>(i + 1));
    LOG_SENSOR(prefix, "X", target.x);
    LOG_SENSOR(prefix, "Y", target.y);
    LOG_SENSOR(prefix, "Distance", target.distance);
    LOG_SENSOR(prefix, "Angle", target.angle);
  }
}

void LD2460Component::loop() {
  const uint32_t now = millis();

  if (now > 2000 && !this->startup_commands_sent_ && !this->reporting_restore_pending_) {
    // Keep the UART rate that is already receiving reports; baud scanning is
    // only useful when there is no data at all.
    if (this->total_bytes_ == 0)
      this->select_next_baud_rate_();
    this->send_startup_commands_();
    this->startup_commands_sent_ = true;
    this->last_command_ms_ = now;
  }

  // Never leave a radar with reporting disabled if a metadata reply is lost.
  if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_METADATA &&
      now - this->last_command_ms_ >= 2000) {
    ESP_LOGW(TAG, "Timed out waiting for LD2460 metadata.");
    if (this->restore_reporting_after_metadata_) {
      this->startup_command_state_ = StartupCommandState::WAITING_FOR_ENABLE;
      this->send_enable_reporting_command_(true);
      this->last_command_ms_ = now;
    } else {
      this->startup_command_state_ = StartupCommandState::COMPLETE;
    }
  }

  // A failed or lost settings acknowledgement must not leave live reporting
  // disabled.  The device is still usable even if the requested change failed.
  if (this->settings_command_state_ != SettingsCommandState::IDLE &&
      now - this->last_command_ms_ >= 2000) {
    ESP_LOGW(TAG, "Timed out waiting for LD2460 settings command 0x%02X.", this->pending_settings_command_);
    this->finish_settings_transaction_(false);
  }

  uint8_t byte;
  size_t bytes_read = 0;
  while (bytes_read < MAX_BYTES_PER_LOOP && this->available() > 0) {
    if (!this->read_byte(&byte))
      break;

    this->rx_buffer_.push_back(byte);
    bytes_read++;
    this->total_bytes_++;
    this->last_byte_ms_ = now;

    // Parse in batches instead of after every byte.  Besides doing less work,
    // this keeps the vector at its reserved size during a UART burst.
    if (this->rx_buffer_.size() >= this->max_buffer_size_) {
      this->process_rx_buffer_();
      if (this->rx_buffer_.size() >= this->max_buffer_size_)
        this->flush_unparsed_buffer_();
    }
  }

  this->process_rx_buffer_();

  if (!this->rx_buffer_.empty() && now - this->last_byte_ms_ >= this->flush_timeout_ms_)
    this->flush_unparsed_buffer_();

  if (this->total_bytes_ == 0 && this->no_data_log_interval_ms_ > 0 &&
      now - this->last_no_data_log_ms_ >= this->no_data_log_interval_ms_) {
    ESP_LOGW(TAG, "No UART bytes received yet on RX. Check LD2460 TX -> ESP RX, common GND, power, and baud.");
    this->last_no_data_log_ms_ = now;
  }
}

void LD2460Component::send_startup_commands_() {
  // The radar does not reliably answer metadata queries while live reporting
  // is active. Stop reports first and restore them after the replies arrive.
  this->restore_reporting_after_metadata_ = this->reporting_enabled_;
  if (this->reporting_enabled_) {
    this->startup_command_state_ = StartupCommandState::WAITING_FOR_DISABLE;
    this->send_enable_reporting_command_(false);
  } else {
    this->startup_command_state_ = StartupCommandState::WAITING_FOR_METADATA;
    this->send_query_version_command_();
    this->send_query_installation_mode_command_();
    this->send_query_installation_parameters_command_();
    this->send_query_detection_range_command_();
    this->last_command_ms_ = millis();
  }
}

void LD2460Component::set_reporting(bool enabled) { this->send_enable_reporting_command_(enabled); }

void LD2460Component::restore_reporting(bool enabled) {
  this->reporting_enabled_ = enabled;
  this->reporting_restore_pending_ = true;
  this->set_reporting(enabled);
}

void LD2460Component::set_config_value(uint8_t field, float value) {
  if (!this->configuration_loaded_) {
    ESP_LOGW(TAG, "Ignoring settings change until current LD2460 settings have been read.");
    return;
  }
  if (this->settings_command_state_ != SettingsCommandState::IDLE) {
    ESP_LOGW(TAG, "Ignoring settings change while another LD2460 settings command is pending.");
    return;
  }
  const bool top_mount = this->installation_mode_ == 2;
  if ((top_mount && (field == 3 || field == 4) && (value < 0.0f || value > 360.0f)) ||
      (!top_mount && (field == 3 || field == 4) && (value < -60.0f || value > 60.0f))) {
    ESP_LOGW(TAG, "%s-mount detection angles are outside the supported range.", top_mount ? "Top" : "Side");
    return;
  }
  if (field == 3 && value >= this->detection_end_angle_deg_) {
    ESP_LOGW(TAG, "Start angle must be less than end angle.");
    return;
  }
  if (field == 4 && value <= this->detection_start_angle_deg_) {
    ESP_LOGW(TAG, "End angle must be greater than start angle.");
    return;
  }
  switch (field) {
    case 0: this->installation_height_m_ = value; this->pending_settings_command_ = 0x07; break;
    case 1: this->installation_angle_deg_ = value; this->pending_settings_command_ = 0x07; break;
    case 2: this->detection_distance_m_ = value; this->pending_settings_command_ = 0x11; break;
    case 3: this->detection_start_angle_deg_ = value; this->pending_settings_command_ = 0x11; break;
    case 4: this->detection_end_angle_deg_ = value; this->pending_settings_command_ = 0x11; break;
    default: return;
  }
  this->restore_reporting_after_settings_ = this->reporting_enabled_;
  if (this->reporting_enabled_) {
    this->settings_command_state_ = SettingsCommandState::WAITING_FOR_DISABLE;
    this->send_enable_reporting_command_(false);
  } else {
    this->settings_command_state_ = SettingsCommandState::WAITING_FOR_ACK;
    if (this->pending_settings_command_ == 0x07)
      this->send_installation_parameters_();
    else if (this->pending_settings_command_ == 0x11)
      this->send_detection_range_();
    else
      this->send_installation_mode_command_(this->pending_installation_mode_);
  }
  this->last_command_ms_ = millis();
}

void LD2460Component::set_installation_mode(const std::string &value) {
  if (this->settings_command_state_ != SettingsCommandState::IDLE) {
    ESP_LOGW(TAG, "Ignoring installation-mode change while a settings command is pending.");
    return;
  }
  this->pending_installation_mode_ = value == "Top" ? 2 : 1;
  this->pending_settings_command_ = 0x09;
  this->restore_reporting_after_settings_ = this->reporting_enabled_;
  if (this->reporting_enabled_) {
    this->settings_command_state_ = SettingsCommandState::WAITING_FOR_DISABLE;
    this->send_enable_reporting_command_(false);
  } else {
    this->settings_command_state_ = SettingsCommandState::WAITING_FOR_ACK;
    this->send_installation_mode_command_(this->pending_installation_mode_);
  }
  this->last_command_ms_ = millis();
}

void LD2460Component::send_installation_mode_command_(uint8_t mode) {
  const uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x09, 0x0C, 0x00, mode, 0x04, 0x03, 0x02, 0x01};
  this->write_array(command, sizeof(command));
  this->flush();
}

void LD2460Component::send_installation_parameters_() {
  const int16_t height = static_cast<int16_t>(std::lround(this->installation_height_m_ * 100.0f));
  const int16_t angle = static_cast<int16_t>(std::lround(this->installation_angle_deg_ * 100.0f));
  const uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x07, 0x0F, 0x00,
                             static_cast<uint8_t>(height), static_cast<uint8_t>(height >> 8),
                             static_cast<uint8_t>(angle), static_cast<uint8_t>(angle >> 8), 0x04, 0x03, 0x02, 0x01};
  this->write_array(command, sizeof(command)); this->flush();
}

void LD2460Component::send_detection_range_() {
  const uint8_t distance = static_cast<uint8_t>(std::lround(this->detection_distance_m_ * 10.0f));
  const int16_t start = static_cast<int16_t>(std::lround(this->detection_start_angle_deg_ * 10.0f));
  const int16_t end = static_cast<int16_t>(std::lround(this->detection_end_angle_deg_ * 10.0f));
  const uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x11, 0x10, 0x00, distance,
                             static_cast<uint8_t>(start), static_cast<uint8_t>(start >> 8),
                             static_cast<uint8_t>(end), static_cast<uint8_t>(end >> 8), 0x04, 0x03, 0x02, 0x01};
  this->write_array(command, sizeof(command)); this->flush();
}

void LD2460Component::publish_config_values_() {
  const float values[] = {this->installation_height_m_, this->installation_angle_deg_, this->detection_distance_m_,
                          this->detection_start_angle_deg_, this->detection_end_angle_deg_};
  for (uint8_t i = 0; i < 5; i++) if (this->config_numbers_[i] != nullptr) this->config_numbers_[i]->publish_state(values[i]);
}

void LD2460Component::send_enable_reporting_command_(bool enabled) {
  uint8_t command[] = {
      0xFD, 0xFC, 0xFB, 0xFA,  // Frame header
      0x06,                    // Open/close reporting function
      0x0C, 0x00,              // Total frame length, little endian
      static_cast<uint8_t>(enabled ? 0x01 : 0x00),  // Enable/disable reporting
      0x04, 0x03, 0x02, 0x01   // Frame tail
  };
  this->write_array(command, sizeof(command));
  this->flush();
  ESP_LOGD(TAG, "Sent LD2460 %s-reporting command.", enabled ? "enable" : "disable");
}

void LD2460Component::send_query_version_command_() {
  static const uint8_t QUERY_VERSION[] = {
      0xFD, 0xFC, 0xFB, 0xFA,  // Frame header
      0x0B,                    // Query firmware version
      0x0C, 0x00,              // Total frame length, little endian
      0x01,                    // Query payload
      0x04, 0x03, 0x02, 0x01   // Frame tail
  };
  this->write_array(QUERY_VERSION, sizeof(QUERY_VERSION));
  this->flush();
  ESP_LOGI(TAG, "Sent LD2460 query-version command.");
}

void LD2460Component::send_query_installation_mode_command_() {
  static const uint8_t QUERY_INSTALLATION_MODE[] = {
      0xFD, 0xFC, 0xFB, 0xFA,  // Frame header
      0x0A,                    // Query installation mode
      0x0C, 0x00,              // Total frame length, little endian
      0x01,                    // Query payload
      0x04, 0x03, 0x02, 0x01   // Frame tail
  };
  this->write_array(QUERY_INSTALLATION_MODE, sizeof(QUERY_INSTALLATION_MODE));
  this->flush();
  ESP_LOGI(TAG, "Sent LD2460 query-installation-mode command.");
}

void LD2460Component::send_query_installation_parameters_command_() {
  static const uint8_t COMMAND[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x0C, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};
  this->write_array(COMMAND, sizeof(COMMAND));
  this->flush();
}

void LD2460Component::send_query_detection_range_command_() {
  static const uint8_t COMMAND[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x12, 0x0C, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};
  this->write_array(COMMAND, sizeof(COMMAND));
  this->flush();
}

void LD2460Component::finish_metadata_queries_() {
  if (this->startup_command_state_ != StartupCommandState::WAITING_FOR_METADATA ||
      !this->firmware_response_received_ || !this->installation_mode_response_received_ ||
      !this->installation_parameters_response_received_ || !this->detection_range_response_received_)
    return;

  if (this->restore_reporting_after_metadata_) {
    this->startup_command_state_ = StartupCommandState::WAITING_FOR_ENABLE;
    this->send_enable_reporting_command_(true);
    this->last_command_ms_ = millis();
  } else {
    this->startup_command_state_ = StartupCommandState::COMPLETE;
  }
}

void LD2460Component::finish_settings_transaction_(bool success) {
  if (!success)
    ESP_LOGW(TAG, "LD2460 settings command 0x%02X failed.", this->pending_settings_command_);
  if (this->restore_reporting_after_settings_) {
    this->settings_command_state_ = SettingsCommandState::WAITING_FOR_ENABLE;
    this->send_enable_reporting_command_(true);
    this->last_command_ms_ = millis();
  } else {
    this->settings_command_state_ = SettingsCommandState::IDLE;
  }
}

void LD2460Component::update_angle_limits_() {
  ESP_LOGI(TAG, "LD2460 %s mount: detection-angle limits are %s.", this->installation_mode_ == 2 ? "top" : "side",
           this->installation_mode_ == 2 ? "0 to 360 degrees" : "-60 to 60 degrees");
}

void LD2460Component::select_next_baud_rate_() {
  if (!this->baud_scan_ && this->startup_commands_sent_)
    return;

  const uint32_t baud_rate = BAUD_RATES[this->baud_index_];
  this->parent_->set_baud_rate(baud_rate);
  this->parent_->load_settings(false);
  ESP_LOGI(TAG, "Testing LD2460 UART baud rate: %" PRIu32, baud_rate);

  if (this->baud_scan_)
    this->baud_index_ = (this->baud_index_ + 1) % (sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]));
}

void LD2460Component::set_target_x_sensor(uint8_t index, sensor::Sensor *target_x_sensor) {
  if (index < MAX_TARGETS)
    this->target_sensors_[index].x = target_x_sensor;
}

void LD2460Component::set_target_y_sensor(uint8_t index, sensor::Sensor *target_y_sensor) {
  if (index < MAX_TARGETS)
    this->target_sensors_[index].y = target_y_sensor;
}

void LD2460Component::set_target_distance_sensor(uint8_t index, sensor::Sensor *target_distance_sensor) {
  if (index < MAX_TARGETS)
    this->target_sensors_[index].distance = target_distance_sensor;
}

void LD2460Component::set_target_angle_sensor(uint8_t index, sensor::Sensor *target_angle_sensor) {
  if (index < MAX_TARGETS)
    this->target_sensors_[index].angle = target_angle_sensor;
}

void LD2460Component::process_rx_buffer_() {
  while (this->rx_buffer_.size() >= 4) {
    if (!is_report_header_(this->rx_buffer_) && !is_command_header_(this->rx_buffer_)) {
      this->rx_buffer_.erase(this->rx_buffer_.begin());
      continue;
    }

    if (this->rx_buffer_.size() < 7)
      return;

    const uint16_t frame_length = read_u16_le_(this->rx_buffer_, 5);
    if (frame_length < 11 || frame_length > this->max_buffer_size_) {
      ESP_LOGW(TAG, "Invalid LD2460 frame length %u in: %s", frame_length,
               format_frame_(this->rx_buffer_).c_str());
      this->rx_buffer_.erase(this->rx_buffer_.begin());
      continue;
    }

    if (this->rx_buffer_.size() < frame_length)
      return;

    // Reuse one allocation for every frame.  Radar reports arrive continuously,
    // so allocating and freeing a vector per report fragments a small MCU heap.
    this->frame_buffer_.assign(this->rx_buffer_.begin(), this->rx_buffer_.begin() + frame_length);
    this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + frame_length);
    const auto &frame = this->frame_buffer_;

    if (is_report_header_(frame)) {
      if (!has_report_footer_(frame)) {
        ESP_LOGW(TAG, "LD2460 report with invalid footer: %s", format_frame_(frame).c_str());
        continue;
      }
      this->process_report_frame_(frame);
      continue;
    }

    if (is_command_header_(frame)) {
      if (!has_command_footer_(frame)) {
        ESP_LOGW(TAG, "LD2460 command frame with invalid footer: %s", format_frame_(frame).c_str());
        continue;
      }
      this->process_command_frame_(frame);
    }
  }
}

void LD2460Component::process_report_frame_(const std::vector<uint8_t> &frame) {
  const uint8_t function_code = frame[4];
  if (function_code != 0x04) {
    ESP_LOGD(TAG, "LD2460 report-like frame function=0x%02X: %s", function_code, format_frame_(frame).c_str());
    return;
  }

  const uint16_t frame_length = read_u16_le_(frame, 5);
  if (frame_length < 11 || (frame_length - 11) % 4 != 0) {
    ESP_LOGW(TAG, "LD2460 target report has invalid length %u: %s", frame_length, format_frame_(frame).c_str());
    return;
  }

  uint8_t target_count = static_cast<uint8_t>((frame_length - 11) / 4);
  if (target_count > MAX_TARGETS) {
    ESP_LOGW(TAG, "LD2460 reported %u targets; only %u are exposed", target_count, MAX_TARGETS);
    target_count = MAX_TARGETS;
  }

  Target targets[MAX_TARGETS]{};
  for (uint8_t i = 0; i < target_count; i++) {
    const size_t offset = 7 + i * 4;
    targets[i].raw_x = read_i16_le_(frame, offset);
    targets[i].raw_y = read_i16_le_(frame, offset + 2);
  }

  const uint32_t now = millis();
  const bool should_publish = this->target_state_changed_(targets, target_count) &&
                              now - this->last_publish_ms_ >= this->publish_interval_ms_;

  // Coordinate conversion, trigonometry, and summary formatting are only
  // useful for a log or state publish. Most reports need only the raw values
  // above to determine whether state changed.
  if (!should_publish) {
    ESP_LOGD(TAG, "LD2460 report raw: %s", format_frame_(frame).c_str());
    return;
  }

  auto &summary = this->summary_buffer_;
  summary.clear();
  char target_count_summary[16];
  std::snprintf(target_count_summary, sizeof(target_count_summary), "targets=%u", target_count);
  summary = target_count_summary;

  for (uint8_t i = 0; i < target_count; i++) {
    const float x_m = targets[i].raw_x / 10.0f;
    const float y_m = targets[i].raw_y / 10.0f;
    const float distance_m = std::sqrt(x_m * x_m + y_m * y_m);
    const float angle_deg = std::atan2(x_m, y_m) * RAD_TO_DEG;

    targets[i].x_m = x_m;
    targets[i].y_m = y_m;
    targets[i].distance_m = distance_m;
    targets[i].angle_deg = angle_deg;

    char target_summary[96];
    std::snprintf(target_summary, sizeof(target_summary), "; T%u x=%.1fm y=%.1fm d=%.1fm angle=%.1fdeg",
                  static_cast<unsigned>(i + 1), x_m, y_m, distance_m, angle_deg);
    summary += target_summary;
  }

  ESP_LOGD(TAG, "LD2460 report raw: %s", format_frame_(frame).c_str());

  if (should_publish) {
    this->publish_targets_(targets, target_count, summary);
    this->remember_published_targets_(targets, target_count);

    this->last_publish_ms_ = now;
  }
}

void LD2460Component::process_command_frame_(const std::vector<uint8_t> &frame) {
  const uint8_t function_code = frame[4];
  const uint16_t frame_length = read_u16_le_(frame, 5);
  const size_t payload_length = frame_length - 11;
  const size_t payload_offset = 7;

  switch (function_code) {
    case 0x06: {
      if (payload_length < 1)
        break;
      const uint8_t result = frame[payload_offset];
      const bool success = (result & 0x10) != 0;
      const bool enabled = (result & 0x01) != 0;
      const bool temporary_startup_disable =
          this->startup_command_state_ == StartupCommandState::WAITING_FOR_DISABLE && !enabled;
      const bool temporary_settings_disable =
          this->settings_command_state_ == SettingsCommandState::WAITING_FOR_DISABLE && !enabled;
      ESP_LOGI(TAG, "LD2460 reporting %s: %s", enabled ? "enable" : "disable", success ? "success" : "failed");
      if (success) {
        this->reporting_enabled_ = enabled;
        this->reporting_restore_pending_ = false;
        if (this->reporting_switch_ != nullptr)
          this->reporting_switch_->publish_state(enabled);

        if (!enabled && !temporary_startup_disable && !temporary_settings_disable)
          this->clear_tracking_states_();

        if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_DISABLE && !enabled) {
          this->startup_command_state_ = StartupCommandState::WAITING_FOR_METADATA;
          this->send_query_version_command_();
          this->send_query_installation_mode_command_();
          this->send_query_installation_parameters_command_();
          this->send_query_detection_range_command_();
          this->last_command_ms_ = millis();
        } else if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_ENABLE && enabled) {
          this->startup_command_state_ = StartupCommandState::COMPLETE;
        } else if (temporary_settings_disable) {
          this->settings_command_state_ = SettingsCommandState::WAITING_FOR_ACK;
          if (this->pending_settings_command_ == 0x07)
            this->send_installation_parameters_();
          else if (this->pending_settings_command_ == 0x11)
            this->send_detection_range_();
          else
            this->send_installation_mode_command_(this->pending_installation_mode_);
          this->last_command_ms_ = millis();
        } else if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ENABLE && enabled) {
          this->settings_command_state_ = SettingsCommandState::IDLE;
        }
      }
      break;
    }
    case 0x0A: {
      if (payload_length < 1)
        break;
      const char *mode = installation_mode_to_string_(frame[payload_offset]);
      this->installation_mode_ = frame[payload_offset];
      ESP_LOGI(TAG, "LD2460 installation mode: %s", mode);
      if (this->installation_mode_text_sensor_ != nullptr)
        this->installation_mode_text_sensor_->publish_state(mode);
      this->installation_mode_response_received_ = true;
      this->update_angle_limits_();
      this->finish_metadata_queries_();
      break;
    }
    case 0x08: {
      if (payload_length < 4)
        break;
      this->installation_height_m_ = read_i16_le_(frame, payload_offset) / 100.0f;
      this->installation_angle_deg_ = read_i16_le_(frame, payload_offset + 2) / 100.0f;
      this->installation_parameters_response_received_ = true;
      this->configuration_loaded_ = this->detection_range_response_received_;
      this->publish_config_values_();
      this->finish_metadata_queries_();
      break;
    }
    case 0x12: {
      if (payload_length < 5)
        break;
      this->detection_distance_m_ = frame[payload_offset] / 10.0f;
      this->detection_start_angle_deg_ = read_i16_le_(frame, payload_offset + 1) / 10.0f;
      this->detection_end_angle_deg_ = read_i16_le_(frame, payload_offset + 3) / 10.0f;
      this->detection_range_response_received_ = true;
      this->configuration_loaded_ = this->installation_parameters_response_received_;
      this->publish_config_values_();
      this->finish_metadata_queries_();
      break;
    }
    case 0x07:
    case 0x11: {
      if (payload_length < 1)
        break;
      const bool success = frame[payload_offset] == 0x01;
      if (success)
        this->publish_config_values_();
      else
        ESP_LOGW(TAG, "LD2460 configuration command 0x%02X failed.", function_code);
      if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ACK &&
          this->pending_settings_command_ == function_code)
        this->finish_settings_transaction_(success);
      break;
    }
    case 0x09: {
      if (payload_length < 1)
        break;
      const uint8_t result = frame[payload_offset];
      if ((result & 0x10) != 0) {
        this->installation_mode_ = result & 0x0F;
        if (this->installation_mode_select_ != nullptr)
          this->installation_mode_select_->publish_state(this->installation_mode_ == 2 ? "Top" : "Side");
        this->update_angle_limits_();
        if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ACK &&
            this->pending_settings_command_ == 0x09)
          this->finish_settings_transaction_(true);
      } else {
        ESP_LOGW(TAG, "LD2460 installation-mode setting failed.");
        if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ACK &&
            this->pending_settings_command_ == 0x09)
          this->finish_settings_transaction_(false);
      }
      break;
    }
    case 0x0B: {
      if (payload_length < 5)
        break;
      const char *mode = installation_mode_to_string_(frame[payload_offset]);
      this->installation_mode_ = frame[payload_offset];
      const uint8_t year = frame[payload_offset + 1];
      const uint8_t month = frame[payload_offset + 2];
      const uint8_t major = frame[payload_offset + 3];
      const uint8_t minor = frame[payload_offset + 4];

      char firmware[48];
      std::snprintf(firmware, sizeof(firmware), "%s V%u.%u (20%02u-%02u)", mode, major, minor, year, month);
      ESP_LOGI(TAG, "LD2460 firmware: %s", firmware);
      if (this->firmware_text_sensor_ != nullptr)
        this->firmware_text_sensor_->publish_state(firmware);
      if (this->installation_mode_text_sensor_ != nullptr)
        this->installation_mode_text_sensor_->publish_state(mode);
      if (this->installation_mode_select_ != nullptr)
        this->installation_mode_select_->publish_state(this->installation_mode_ == 2 ? "Top" : "Side");
      this->firmware_response_received_ = true;
      this->finish_metadata_queries_();
      break;
    }
    default:
      ESP_LOGI(TAG, "LD2460 command/ack function=0x%02X payload=%u byte(s): %s", function_code,
               static_cast<unsigned>(payload_length), format_frame_(frame).c_str());
      break;
  }

}

void LD2460Component::publish_targets_(const Target *targets, uint8_t target_count, const std::string &summary) {
  const bool count_changed = !this->has_published_targets_ || target_count != this->last_published_target_count_;

  if (count_changed) {
    if (this->presence_binary_sensor_ != nullptr)
      this->presence_binary_sensor_->publish_state(target_count > 0);

    if (this->target_count_sensor_ != nullptr)
      this->target_count_sensor_->publish_state(target_count);
  }

  if (this->summary_text_sensor_ != nullptr)
    this->summary_text_sensor_->publish_state(summary);

  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    const bool present = i < target_count;
    const bool was_present = this->has_published_targets_ && i < this->last_published_target_count_;
    const bool coordinates_changed =
        present && (!was_present || targets[i].raw_x != this->last_published_targets_[i].raw_x ||
                    targets[i].raw_y != this->last_published_targets_[i].raw_y);

    // Publish only slots that appeared, moved, or disappeared. Movement of one
    // target must not republish every other occupied target.
    if (!coordinates_changed && !(was_present && !present))
      continue;

    const float x = present ? targets[i].x_m : NAN;
    const float y = present ? targets[i].y_m : NAN;
    const float distance = present ? targets[i].distance_m : NAN;
    const float angle = present ? targets[i].angle_deg : NAN;
    const auto target = this->target_sensors_[i];

    if (target.x != nullptr)
      target.x->publish_state(x);
    if (target.y != nullptr)
      target.y->publish_state(y);
    if (target.distance != nullptr)
      target.distance->publish_state(distance);
    if (target.angle != nullptr)
      target.angle->publish_state(angle);
  }
}

void LD2460Component::clear_tracking_states_() {
  if (this->presence_binary_sensor_ != nullptr)
    this->presence_binary_sensor_->publish_state(false);
  if (this->target_count_sensor_ != nullptr)
    this->target_count_sensor_->publish_state(0);
  if (this->summary_text_sensor_ != nullptr)
    this->summary_text_sensor_->publish_state("targets=0");

  for (auto &target : this->target_sensors_) {
    if (target.x != nullptr)
      target.x->publish_state(NAN);
    if (target.y != nullptr)
      target.y->publish_state(NAN);
    if (target.distance != nullptr)
      target.distance->publish_state(NAN);
    if (target.angle != nullptr)
      target.angle->publish_state(NAN);
  }

  this->last_published_target_count_ = 0;
  this->has_published_targets_ = false;
}

bool LD2460Component::target_state_changed_(const Target *targets, uint8_t target_count) const {
  if (!this->has_published_targets_)
    return true;

  if (target_count != this->last_published_target_count_)
    return true;

  for (uint8_t i = 0; i < target_count; i++) {
    if (targets[i].raw_x != this->last_published_targets_[i].raw_x ||
        targets[i].raw_y != this->last_published_targets_[i].raw_y) {
      return true;
    }
  }

  return false;
}

void LD2460Component::remember_published_targets_(const Target *targets, uint8_t target_count) {
  this->last_published_target_count_ = target_count;
  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    this->last_published_targets_[i] = i < target_count ? targets[i] : Target{};
  }
  this->has_published_targets_ = true;
}

void LD2460Component::flush_unparsed_buffer_() {
  if (this->rx_buffer_.empty())
    return;

  const std::string frame = this->format_frame_(this->rx_buffer_);
  ESP_LOGW(TAG, "Unparsed RX %u byte(s): %s", static_cast<unsigned>(this->rx_buffer_.size()), frame.c_str());


  this->rx_buffer_.clear();
}

bool LD2460Component::is_report_header_(const std::vector<uint8_t> &bytes) {
  return bytes.size() >= 4 && bytes[0] == 0xF4 && bytes[1] == 0xF3 && bytes[2] == 0xF2 && bytes[3] == 0xF1;
}

bool LD2460Component::is_command_header_(const std::vector<uint8_t> &bytes) {
  return bytes.size() >= 4 && bytes[0] == 0xFD && bytes[1] == 0xFC && bytes[2] == 0xFB && bytes[3] == 0xFA;
}

bool LD2460Component::has_report_footer_(const std::vector<uint8_t> &bytes) {
  return bytes.size() >= 4 && bytes[bytes.size() - 4] == 0xF8 && bytes[bytes.size() - 3] == 0xF7 &&
         bytes[bytes.size() - 2] == 0xF6 && bytes[bytes.size() - 1] == 0xF5;
}

bool LD2460Component::has_command_footer_(const std::vector<uint8_t> &bytes) {
  return bytes.size() >= 4 && bytes[bytes.size() - 4] == 0x04 && bytes[bytes.size() - 3] == 0x03 &&
         bytes[bytes.size() - 2] == 0x02 && bytes[bytes.size() - 1] == 0x01;
}

uint16_t LD2460Component::read_u16_le_(const std::vector<uint8_t> &bytes, size_t index) {
  return static_cast<uint16_t>(bytes[index]) | (static_cast<uint16_t>(bytes[index + 1]) << 8);
}

int16_t LD2460Component::read_i16_le_(const std::vector<uint8_t> &bytes, size_t index) {
  return static_cast<int16_t>(read_u16_le_(bytes, index));
}

const char *LD2460Component::installation_mode_to_string_(uint8_t mode) {
  switch (mode) {
    case 0x01:
      return "side";
    case 0x02:
      return "top";
    default:
      return "unknown";
  }
}

std::string LD2460Component::format_frame_(const std::vector<uint8_t> &bytes) {
  std::string output;
  output.reserve(bytes.size() * 4 + 3);

  char byte_hex[3];
  for (size_t i = 0; i < bytes.size(); i++) {
    if (i != 0)
      output += ' ';
    std::snprintf(byte_hex, sizeof(byte_hex), "%02X", bytes[i]);
    output += byte_hex;
  }

  output += " | ";
  for (const auto byte : bytes) {
    if (std::isprint(static_cast<unsigned char>(byte)))
      output += static_cast<char>(byte);
    else
      output += '.';
  }

  return output;
}

}  // namespace ld2460
}  // namespace esphome
