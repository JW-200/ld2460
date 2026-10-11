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
static const uint32_t COMMAND_TIMEOUT_MS = 750;
static const uint8_t MAX_COMMAND_RETRIES = 2;
// Do not let a continuously busy UART monopolize ESPHome's main loop.  The
// remaining bytes stay in the UART driver's ring buffer for the next loop.
static const size_t MAX_BYTES_PER_LOOP = 128;
static const char *const EXIT_EDGE_OPTIONS[] = {"Disabled", "P1-P2", "P2-P3", "P3-P4", "P4-P1"};

void LD2460TrackingNumber::control(float value) {
  if (this->parent_ != nullptr)
    this->parent_->set_tracking_value(this->field_, value);
}

void LD2460ExitEdgeSelect::control(const std::string &value) {
  if (this->parent_ == nullptr)
    return;
  for (uint8_t i = 0; i < 5; i++) {
    if (value == EXIT_EDGE_OPTIONS[i]) {
      this->parent_->set_tracking_value(8 + this->slot_, i);
      return;
    }
  }
}

void LD2460ZoneSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_zone_enabled(state);
}

void LD2460ReportingSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_reporting(state);
}

void LD2460ReportingSwitch::setup() {
  const auto restored_state = this->get_initial_state_with_restore_mode();
  const bool enabled = restored_state.value_or(true);
  if (this->parent_ != nullptr)
    this->parent_->restore_reporting(enabled);
  // Always initialize the entity. Waiting for a transport acknowledgement
  // left the switch unavailable when the user's restored choice was OFF.
  this->publish_state(enabled);
}

void LD2460ConfigNumber::control(float value) {
  if (this->parent_ != nullptr)
    this->parent_->set_config_value(this->field_, value);
}

void LD2460InstallationModeSelect::control(const std::string &value) {
  if (this->parent_ != nullptr)
    this->parent_->set_installation_mode(value);
}
void LD2460SensitivitySelect::control(const std::string &value) {
  if (this->parent_ != nullptr)
    this->parent_->set_sensitivity(value);
}

void LD2460Component::setup() {
  this->rx_buffer_.reserve(this->max_buffer_size_);
  this->frame_buffer_.reserve(this->max_buffer_size_);
  this->tracking_preference_ = global_preferences->make_preference<TrackingSettings>(this->tracking_preference_key_);
  const auto defaults = this->tracking_settings_;
  if (this->tracking_preference_.load(&this->tracking_settings_) && !this->valid_tracking_settings_()) {
    ESP_LOGW(TAG, "Ignoring invalid saved tracking settings.");
    this->tracking_settings_ = defaults;
  }
  this->apply_tracking_settings_(false);
  this->publish_tracking_settings_();
}

void LD2460Component::set_tracking_preference_key(const std::string &id) {
  this->tracking_preference_key_ = fnv1_hash("ld2460:tracking:v1:" + id);
}

bool LD2460Component::valid_tracking_settings_() const {
  const auto &v = this->tracking_settings_.values;
  for (uint8_t i = 0; i < TRACKING_FIELDS; i++) {
    if (!std::isfinite(v[i]))
      return false;
    if (i < 8 && (v[i] < -12 || v[i] > 12))
      return false;
    if ((i == 8 || i == 9) && (v[i] < 0 || v[i] > 4 || v[i] != std::floor(v[i])))
      return false;
  }
  if (v[10] < 1 || v[10] > 3600 || v[11] < v[10] || v[11] > 86400 ||
      v[12] < 1 || v[12] > 300 || v[13] < 1 || v[13] > 120 || v[14] < 0.1f || v[14] > 2)
    return false;
  if (this->tracking_settings_.zone_enabled) {
    PresenceTracker::Point vertices[4]{};
    for (uint8_t i = 0; i < 4; i++) {
      vertices[i].x = static_cast<int16_t>(std::lround(v[i * 2] * 10));
      vertices[i].y = static_cast<int16_t>(std::lround(v[i * 2 + 1] * 10));
    }
    return TrackingGeometry::valid_zone(vertices);
  }
  return true;
}

void LD2460Component::apply_tracking_settings_(bool geometry_changed) {
  const auto &v = this->tracking_settings_.values;
  this->presence_tracker_.set_presence_timeout(static_cast<uint32_t>(v[10] * 1000));
  this->presence_tracker_.set_stationary_presence_timeout(static_cast<uint32_t>(v[11] * 1000));
  this->presence_tracker_.set_stationary_dwell(static_cast<uint32_t>(v[12] * 1000));
  this->presence_tracker_.set_exit_timeout(static_cast<uint32_t>(v[13] * 1000));
  this->presence_tracker_.set_edge_margin(v[14]);
  if (geometry_changed) {
    PresenceTracker::Point vertices[4]{};
    for (uint8_t i = 0; i < 4; i++) {
      vertices[i].x = static_cast<int16_t>(std::lround(v[i * 2] * 10));
      vertices[i].y = static_cast<int16_t>(std::lround(v[i * 2 + 1] * 10));
    }
    this->presence_tracker_.set_zone(vertices, this->tracking_settings_.zone_enabled);
    this->presence_tracker_.set_exit_edges(static_cast<uint8_t>(v[8]), static_cast<uint8_t>(v[9]));
    this->presence_tracker_.clear();
    this->invalidate_tracking_states_();
  } else if (!this->startup_commands_sent_) {
    PresenceTracker::Point vertices[4]{};
    for (uint8_t i = 0; i < 4; i++) {
      vertices[i].x = static_cast<int16_t>(std::lround(v[i * 2] * 10));
      vertices[i].y = static_cast<int16_t>(std::lround(v[i * 2 + 1] * 10));
    }
    this->presence_tracker_.set_zone(vertices, this->tracking_settings_.zone_enabled);
    this->presence_tracker_.set_exit_edges(static_cast<uint8_t>(v[8]), static_cast<uint8_t>(v[9]));
  }
}

void LD2460Component::publish_tracking_settings_() {
  for (uint8_t i = 0; i < TRACKING_FIELDS; i++) {
    auto *control = this->tracking_numbers_[i];
    if (control != nullptr && (!control->has_state() || control->state != this->tracking_settings_.values[i]))
      control->publish_state(this->tracking_settings_.values[i]);
  }
  for (uint8_t i = 0; i < 2; i++) {
    auto *control = this->exit_edge_selects_[i];
    const char *value = EXIT_EDGE_OPTIONS[static_cast<uint8_t>(this->tracking_settings_.values[8 + i])];
    if (control != nullptr && (!control->has_state() || control->current_option() != value))
      control->publish_state(value);
  }
  if (this->zone_switch_ != nullptr &&
      (!this->zone_switch_->has_state() || this->zone_switch_->state != this->tracking_settings_.zone_enabled))
    this->zone_switch_->publish_state(this->tracking_settings_.zone_enabled);
}

void LD2460Component::set_tracking_value(uint8_t field, float value) {
  if (field >= TRACKING_FIELDS || value == this->tracking_settings_.values[field])
    return;
  const float previous = this->tracking_settings_.values[field];
  this->tracking_settings_.values[field] = value;
  if (!this->valid_tracking_settings_()) {
    this->tracking_settings_.values[field] = previous;
    ESP_LOGW(TAG, "Invalid tracking setting; keep timeouts ordered and disable the room zone before reshaping it.");
    // Send the accepted value back even if our local state never changed.
    if (this->tracking_numbers_[field] != nullptr)
      this->tracking_numbers_[field]->publish_state(previous);
    this->publish_tracking_settings_();
    return;
  }
  this->apply_tracking_settings_(field < 8);
  if (field == 8 || field == 9)
    this->presence_tracker_.set_exit_edges(static_cast<uint8_t>(this->tracking_settings_.values[8]),
                                         static_cast<uint8_t>(this->tracking_settings_.values[9]));
  this->tracking_preference_.save(&this->tracking_settings_);
  this->publish_tracking_settings_();
}

void LD2460Component::set_zone_enabled(bool enabled) {
  if (enabled == this->tracking_settings_.zone_enabled)
    return;
  this->tracking_settings_.zone_enabled = enabled;
  if (!this->valid_tracking_settings_()) {
    this->tracking_settings_.zone_enabled = !enabled;
    ESP_LOGW(TAG, "Room zone must have four distinct corners in perimeter order and no crossed edges.");
    if (this->zone_switch_ != nullptr)
      this->zone_switch_->publish_state(!enabled);
    return;
  }
  this->apply_tracking_settings_(true);
  this->tracking_preference_.save(&this->tracking_settings_);
  this->publish_tracking_settings_();
}

void LD2460Component::update_radar_boundary_() {
  this->presence_tracker_.set_radar_boundary(this->detection_distance_m_, this->detection_start_angle_deg_,
                                            this->detection_end_angle_deg_,
                                            this->detection_range_response_received_ && this->installation_mode_ == 1);
}

void LD2460Component::note_radar_response_() {
  this->last_radar_response_ms_ = millis();
  this->radar_response_seen_ = true;
}

void LD2460Component::service_radar_health_(uint32_t now) {
  const bool healthy = this->radar_response_seen_ && now - this->last_radar_response_ms_ < this->radar_health_timeout_ms_;
  if (!this->radar_availability_known_ || healthy != this->radar_available_) {
    this->radar_available_ = healthy;
    this->radar_availability_known_ = true;
    if (this->radar_available_sensor_ != nullptr)
      this->radar_available_sensor_->publish_state(healthy);
    if (!healthy) {
      this->presence_tracker_.clear();
      this->invalidate_tracking_states_();
    }
  }
  if (this->startup_command_state_ != StartupCommandState::COMPLETE ||
      this->settings_command_state_ != SettingsCommandState::IDLE)
    return;
  this->health_probe_active_ = false;
  if (healthy && this->requested_reporting_enabled_ && this->reporting_enabled_) {
    this->presence_tracker_.age(now);
    this->publish_tracking_state_();
  } else {
    this->invalidate_tracking_states_();
  }
  // Silence is permitted by the radar protocol. Probe only after a minute
  // without valid traffic, using the existing pause/query/restore transaction.
  // This is UART traffic; it publishes no unchanged states to Thread clients.
  if (now - this->last_radar_response_ms_ >= 60000 && now - this->last_health_probe_ms_ >= 60000) {
    this->last_health_probe_ms_ = now;
    this->health_probe_active_ = true;
    this->send_startup_commands_();
  }
}

void LD2460Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2460 raw UART reader:");
  ESP_LOGCONFIG(TAG, "  Flush timeout: %" PRIu32 " ms", this->flush_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Max buffer size: %u byte(s)", this->max_buffer_size_);
  ESP_LOGCONFIG(TAG, "  Baud scan: %s", YESNO(this->baud_scan_));
  ESP_LOGCONFIG(TAG, "  No-data log interval: %" PRIu32 " ms", this->no_data_log_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Publish interval: %" PRIu32 " ms", this->publish_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Position update threshold: %u dm", static_cast<unsigned>(this->position_update_threshold_dm_));
  ESP_LOGCONFIG(TAG, "  Presence timeout: %" PRIu32 " ms", this->presence_tracker_.get_presence_timeout());
  ESP_LOGCONFIG(TAG, "  Stationary dwell: %" PRIu32 " ms", this->presence_tracker_.get_stationary_dwell());
  ESP_LOGCONFIG(TAG, "  Stationary presence timeout: %" PRIu32 " ms",
                this->presence_tracker_.get_stationary_presence_timeout());
  this->check_uart_settings(115200, 1, uart::UART_CONFIG_PARITY_NONE, 8);
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

  // Track aging is independent of radar frame arrival and network publication.
  this->presence_tracker_.age(millis());

  if (!this->rx_buffer_.empty() && now - this->last_byte_ms_ >= this->flush_timeout_ms_)
    this->flush_unparsed_buffer_();

  if (this->total_bytes_ == 0 && this->no_data_log_interval_ms_ > 0 &&
      now - this->last_no_data_log_ms_ >= this->no_data_log_interval_ms_) {
    ESP_LOGW(TAG, "No UART bytes received yet on RX. Check LD2460 TX -> ESP RX, common GND, power, and baud.");
    this->last_no_data_log_ms_ = now;
  }

  // Handle replies before checking timeouts. This avoids retransmitting a
  // command whose acknowledgement was already waiting in the UART buffer.
  if (now > 2000 && !this->startup_commands_sent_) {
    if (this->total_bytes_ == 0)
      this->select_next_baud_rate_();
    this->send_startup_commands_();
    this->startup_commands_sent_ = true;
    this->last_command_ms_ = now;
  }
  this->service_startup_transaction_(now);
  this->service_settings_transaction_(now);
  this->service_radar_health_(millis());
}

void LD2460Component::send_startup_commands_() {
  // The radar does not reliably answer metadata queries while live reporting
  // is active. Stop reports first and restore the saved user choice after the
  // replies arrive.  In particular, do not send this command from the switch
  // setup hook: that hook can run before the UART driver is ready on ESP32.
  this->restore_reporting_after_metadata_ = this->requested_reporting_enabled_;
  this->startup_queries_sent_ = false;
  this->startup_query_waiting_ = false;
  this->startup_query_index_ = 0;
  this->startup_expected_response_function_ = 0;
  this->startup_retry_count_ = 0;
  this->startup_command_state_ = StartupCommandState::WAITING_FOR_DISABLE;
  this->send_enable_reporting_command_(false);
}

void LD2460Component::send_startup_queries_() {
  if (this->startup_command_state_ != StartupCommandState::WAITING_FOR_METADATA) {
    ESP_LOGW(TAG, "Refusing LD2460 startup settings queries while reporting may be active.");
    return;
  }
  if (this->startup_query_waiting_)
    return;

  // Installation parameters exist only for side-mounted units.  Do not issue
  // an unsupported query once the mode response has identified a top mount.
  if (this->startup_query_index_ == 2 && (this->installation_mode_ == 2 ||
      !(this->firmware_response_received_ || this->installation_mode_response_received_)))
    this->startup_query_index_++;

  const uint8_t query_count = this->health_probe_active_ ? 1 : (this->sensitivity_select_ != nullptr ? 5 : 4);
  if (this->startup_query_index_ >= query_count) {
    this->startup_queries_sent_ = true;
    this->finish_metadata_queries_();
    return;
  }

  switch (this->startup_query_index_) {
    case 0:
      this->send_query_version_command_();
      this->startup_expected_response_function_ = 0x0B;
      break;
    case 1:
      this->send_query_installation_mode_command_();
      this->startup_expected_response_function_ = 0x0A;
      break;
    case 2:
      this->send_query_installation_parameters_command_();
      this->startup_expected_response_function_ = 0x08;
      break;
    case 3:
      this->send_query_detection_range_command_();
      this->startup_expected_response_function_ = 0x12;
      break;
    case 4:
      this->send_query_sensitivity_command_();
      this->startup_expected_response_function_ = 0x14;
      break;
    default:
      return;
  }
  this->startup_query_waiting_ = true;
  this->startup_retry_count_ = 0;
  this->last_command_ms_ = millis();
}

void LD2460Component::resend_startup_query_() {
  switch (this->startup_query_index_) {
    case 0: this->send_query_version_command_(); break;
    case 1: this->send_query_installation_mode_command_(); break;
    case 2: this->send_query_installation_parameters_command_(); break;
    case 3: this->send_query_detection_range_command_(); break;
    case 4: this->send_query_sensitivity_command_(); break;
    default: break;
  }
}

void LD2460Component::advance_startup_query_(uint8_t response_function_code) {
  if (this->startup_command_state_ != StartupCommandState::WAITING_FOR_METADATA || !this->startup_query_waiting_ ||
      this->startup_query_index_ >= 5)
    return;

  if (response_function_code != this->startup_expected_response_function_)
    return;

  this->startup_query_waiting_ = false;
  this->startup_expected_response_function_ = 0;
  this->startup_query_index_++;
  this->startup_retry_count_ = 0;
  this->last_command_ms_ = millis();
  if (this->startup_query_index_ >= 5)
    this->startup_queries_sent_ = true;
}

void LD2460Component::service_startup_transaction_(uint32_t now) {
  if (this->startup_command_state_ == StartupCommandState::IDLE ||
      this->startup_command_state_ == StartupCommandState::COMPLETE)
    return;

  if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_METADATA &&
      !this->startup_query_waiting_) {
    if (now - this->last_command_ms_ >= 100)
      this->send_startup_queries_();
    return;
  }

  if (now - this->last_command_ms_ < COMMAND_TIMEOUT_MS)
    return;

  if (this->startup_retry_count_ < MAX_COMMAND_RETRIES) {
    this->startup_retry_count_++;
    ESP_LOGW(TAG, "Retrying LD2460 startup command (%u/%u).",
             this->startup_retry_count_, MAX_COMMAND_RETRIES);
    if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_DISABLE)
      this->send_enable_reporting_command_(false);
    else if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_ENABLE)
      this->send_enable_reporting_command_(this->restore_reporting_after_metadata_);
    else
      this->resend_startup_query_();
    this->last_command_ms_ = now;
    return;
  }

  this->startup_retry_count_ = 0;
  if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_DISABLE) {
    // The command may have succeeded even if its acknowledgement was lost.
    // Continue querying, then make a final best effort to restore reporting.
    ESP_LOGW(TAG, "No acknowledgement for reporting disable; continuing startup queries.");
    this->startup_command_state_ = StartupCommandState::WAITING_FOR_METADATA;
    this->last_command_ms_ = now;
  } else if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_METADATA) {
    ESP_LOGW(TAG, "No response to startup query 0x%02X; continuing with remaining fields.",
             this->startup_expected_response_function_);
    this->startup_query_waiting_ = false;
    this->startup_expected_response_function_ = 0;
    this->startup_query_index_++;
    this->last_command_ms_ = now;
  } else {
    ESP_LOGE(TAG, "No acknowledgement while restoring reporting; startup will continue in degraded state.");
    this->startup_command_state_ = StartupCommandState::COMPLETE;
  }
}

void LD2460Component::set_reporting(bool enabled) {
  if (this->settings_command_state_ != SettingsCommandState::IDLE ||
      (this->startup_command_state_ != StartupCommandState::IDLE &&
       this->startup_command_state_ != StartupCommandState::COMPLETE)) {
    ESP_LOGD(TAG, "Queued reporting change while an LD2460 transaction is active.");
    this->requested_reporting_enabled_ = enabled;
    this->restore_reporting_after_metadata_ = enabled;
    this->restore_reporting_after_settings_ = enabled;
    if (this->reporting_switch_ != nullptr)
      this->reporting_switch_->publish_state(enabled);
    return;
  }
  this->requested_reporting_enabled_ = enabled;
  this->send_enable_reporting_command_(enabled);
}

void LD2460Component::restore_reporting(bool enabled) {
  this->requested_reporting_enabled_ = enabled;
  // Switch setup runs independently of this component's setup priority.  A
  // write/flush here could access a UART that has not been initialized yet.
  // The startup transaction in loop() applies this saved state safely.
}

void LD2460Component::set_config_value(uint8_t field, float value) {
  if (this->startup_command_state_ != StartupCommandState::COMPLETE) {
    ESP_LOGW(TAG, "Ignoring settings change while LD2460 startup is in progress.");
    return;
  }
  const bool installation_setting = field == 0 || field == 1;
  const bool detection_setting = field >= 2 && field <= 4;
  if ((installation_setting && !this->installation_parameters_response_received_) ||
      (detection_setting && !this->detection_range_response_received_)) {
    ESP_LOGW(TAG, "Ignoring settings change until its current LD2460 values have been read.");
    return;
  }
  if (this->settings_command_state_ != SettingsCommandState::IDLE) {
    ESP_LOGW(TAG, "Ignoring settings change while another LD2460 settings command is pending.");
    return;
  }
  if (this->installation_mode_ != 1 && (field == 0 || field == 1)) {
    ESP_LOGW(TAG, "Installation height and angle can only be changed for side mount.");
    return;
  }
  float height = this->installation_height_m_;
  float installation_angle = this->installation_angle_deg_;
  float distance = this->detection_distance_m_;
  float start_angle = this->detection_start_angle_deg_;
  float end_angle = this->detection_end_angle_deg_;
  switch (field) {
    case 0: height = value; break;
    case 1: installation_angle = value; break;
    case 2: distance = value; break;
    case 3: start_angle = value; break;
    case 4: end_angle = value; break;
    default: return;
  }
  if (!this->configuration_values_in_range_(height, installation_angle, distance, start_angle, end_angle)) {
    ESP_LOGW(TAG, "Refusing to send LD2460 settings outside their supported range.");
    return;
  }
  // Keep acknowledged values authoritative until the matching setting ACK.
  this->pending_config_values_[0] = height;
  this->pending_config_values_[1] = installation_angle;
  this->pending_config_values_[2] = distance;
  this->pending_config_values_[3] = start_angle;
  this->pending_config_values_[4] = end_angle;
  this->pending_settings_command_ = installation_setting ? 0x07 : 0x11;
  this->restore_reporting_after_settings_ = this->requested_reporting_enabled_;
  this->settings_retry_count_ = 0;
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
  if (this->startup_command_state_ != StartupCommandState::COMPLETE) {
    ESP_LOGW(TAG, "Ignoring installation-mode change while LD2460 startup is in progress.");
    return;
  }
  if (this->settings_command_state_ != SettingsCommandState::IDLE) {
    ESP_LOGW(TAG, "Ignoring installation-mode change while a settings command is pending.");
    return;
  }
  this->pending_installation_mode_ = value == "Top" ? 2 : 1;
  this->pending_settings_command_ = 0x09;
  this->restore_reporting_after_settings_ = this->requested_reporting_enabled_;
  this->settings_retry_count_ = 0;
  if (this->reporting_enabled_) {
    this->settings_command_state_ = SettingsCommandState::WAITING_FOR_DISABLE;
    this->send_enable_reporting_command_(false);
  } else {
    this->settings_command_state_ = SettingsCommandState::WAITING_FOR_ACK;
    this->send_installation_mode_command_(this->pending_installation_mode_);
  }
  this->last_command_ms_ = millis();
}

void LD2460Component::set_sensitivity(const std::string &value) {
  if (this->startup_command_state_ != StartupCommandState::COMPLETE) {
    ESP_LOGW(TAG, "Ignoring sensitivity change while LD2460 startup is in progress.");
    return;
  }
  if (this->settings_command_state_ != SettingsCommandState::IDLE) {
    ESP_LOGW(TAG, "Ignoring sensitivity change while a settings command is pending.");
    return;
  }
  this->pending_sensitivity_ = value == "High" ? 1 : value == "Medium" ? 2 : 3;
  this->pending_settings_command_ = 0x13;
  this->restore_reporting_after_settings_ = this->requested_reporting_enabled_;
  this->settings_retry_count_ = 0;
  this->settings_command_state_ = this->reporting_enabled_ ? SettingsCommandState::WAITING_FOR_DISABLE
                                                            : SettingsCommandState::WAITING_FOR_ACK;
  if (this->reporting_enabled_)
    this->send_enable_reporting_command_(false);
  else
    this->send_sensitivity_command_(this->pending_sensitivity_);
  this->last_command_ms_ = millis();
}

void LD2460Component::send_installation_mode_command_(uint8_t mode) {
  this->send_protocol_command_(Protocol::command(0x09, {mode}));
}
void LD2460Component::send_sensitivity_command_(uint8_t sensitivity) {
  this->send_protocol_command_(Protocol::command(0x13, {sensitivity}));
}

void LD2460Component::send_installation_parameters_() {
  if (this->installation_mode_ != 1)
    return;
  this->send_protocol_command_(Protocol::installation(
      static_cast<uint16_t>(std::lround(this->pending_config_values_[0] * 100.0f)),
      static_cast<uint16_t>(std::lround(this->pending_config_values_[1] * 100.0f))));
}

void LD2460Component::send_detection_range_() {
  this->send_protocol_command_(Protocol::range(
      static_cast<uint8_t>(std::lround(this->pending_config_values_[2] * 10.0f)),
      static_cast<int16_t>(std::lround(this->pending_config_values_[3] * 10.0f)),
      static_cast<int16_t>(std::lround(this->pending_config_values_[4] * 10.0f))));
}

bool LD2460Component::configuration_values_in_range_(float height, float installation_angle, float distance,
                                                      float start_angle, float end_angle) const {
  if (!std::isfinite(height) || !std::isfinite(installation_angle) || !std::isfinite(distance) ||
      !std::isfinite(start_angle) || !std::isfinite(end_angle) || height < 1.6f || height > 2.6f ||
      installation_angle < 0.0f || installation_angle > 30.0f ||
      distance < 0.0f || distance > (this->installation_mode_ == 2 ? 4.0f : 6.0f) || start_angle >= end_angle)
    return false;

  if (this->installation_mode_ == 2)
    return start_angle >= 0.0f && end_angle <= 360.0f;
  return start_angle >= -60.0f && end_angle <= 60.0f;
}

void LD2460Component::publish_config_values_() {
  const float values[] = {this->installation_height_m_, this->installation_angle_deg_, this->detection_distance_m_,
                          this->detection_start_angle_deg_, this->detection_end_angle_deg_};
  for (uint8_t i = 0; i < 5; i++) {
    const bool value_was_read = i < 2 ? this->installation_parameters_response_received_
                                      : this->detection_range_response_received_;
    if (value_was_read && this->config_numbers_[i] != nullptr &&
        (!this->config_numbers_[i]->has_state() || this->config_numbers_[i]->state != values[i]))
      this->config_numbers_[i]->publish_state(values[i]);
  }
}

void LD2460Component::send_enable_reporting_command_(bool enabled) {
  this->send_protocol_command_(Protocol::command(0x06, {static_cast<uint8_t>(enabled)}));
  ESP_LOGD(TAG, "Sent LD2460 %s-reporting command.", enabled ? "enable" : "disable");
}

void LD2460Component::send_query_version_command_() {
  this->send_protocol_command_(Protocol::command(0x0B, {0x01}));
}

void LD2460Component::send_query_installation_mode_command_() {
  this->send_protocol_command_(Protocol::command(0x0A, {0x01}));
}

void LD2460Component::send_query_installation_parameters_command_() {
  this->send_protocol_command_(Protocol::command(0x08, {0x01}));
}

void LD2460Component::send_query_detection_range_command_() {
  this->send_protocol_command_(Protocol::command(0x12, {0x01}));
}
void LD2460Component::send_query_sensitivity_command_() {
  this->send_protocol_command_(Protocol::command(0x14, {0x01}));
}

void LD2460Component::finish_metadata_queries_() {
  if (this->startup_command_state_ != StartupCommandState::WAITING_FOR_METADATA ||
      !this->startup_queries_sent_)
    return;

  if (!this->health_probe_active_ && (!this->firmware_response_received_ || !this->installation_mode_response_received_ ||
      !(this->installation_mode_ == 2 || this->installation_parameters_response_received_) ||
      !this->detection_range_response_received_ || (this->sensitivity_select_ != nullptr && !this->sensitivity_response_received_))) {
    ESP_LOGW(TAG, "LD2460 startup completed with one or more unavailable metadata fields.");
  }

  // Explicitly apply the restored choice even when it is OFF. This resolves
  // ambiguity after a lost initial disable acknowledgement.
  this->startup_command_state_ = StartupCommandState::WAITING_FOR_ENABLE;
  this->startup_retry_count_ = 0;
  this->send_enable_reporting_command_(this->restore_reporting_after_metadata_);
  this->last_command_ms_ = millis();
}

void LD2460Component::finish_settings_transaction_(bool success) {
  if (!success)
    ESP_LOGW(TAG, "LD2460 settings command 0x%02X failed; reading actual values back.", this->pending_settings_command_);
  if (success && this->pending_settings_command_ == 0x07) {
    this->installation_height_m_ = this->pending_config_values_[0];
    this->installation_angle_deg_ = this->pending_config_values_[1];
  } else if (success && this->pending_settings_command_ == 0x11) {
    this->detection_distance_m_ = this->pending_config_values_[2];
    this->detection_start_angle_deg_ = this->pending_config_values_[3];
    this->detection_end_angle_deg_ = this->pending_config_values_[4];
    this->update_radar_boundary_();
  }
  if (success)
    this->publish_config_values_();
  // A lost ACK does not tell us whether a write took effect. Read back while
  // paused; a mode change also selects a different set of radar settings.
  if (this->pending_settings_command_ == 0x07)
    this->installation_parameters_response_received_ = false;
  if (this->pending_settings_command_ == 0x11)
    this->detection_range_response_received_ = false;
  this->update_radar_boundary_();
  this->settings_command_state_ = SettingsCommandState::IDLE;
  this->health_probe_active_ = false;
  this->restore_reporting_after_metadata_ = this->requested_reporting_enabled_;
  this->startup_queries_sent_ = false;
  this->startup_query_waiting_ = false;
  this->startup_query_index_ = 0;
  this->startup_retry_count_ = 0;
  this->startup_command_state_ = StartupCommandState::WAITING_FOR_METADATA;
  this->last_command_ms_ = millis();
}

void LD2460Component::service_settings_transaction_(uint32_t now) {
  if (this->settings_command_state_ == SettingsCommandState::IDLE ||
      now - this->last_command_ms_ < COMMAND_TIMEOUT_MS)
    return;

  if (this->settings_retry_count_ < MAX_COMMAND_RETRIES) {
    this->settings_retry_count_++;
    ESP_LOGW(TAG, "Retrying LD2460 settings transaction 0x%02X (%u/%u).",
             this->pending_settings_command_, this->settings_retry_count_, MAX_COMMAND_RETRIES);
    if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_DISABLE) {
      this->send_enable_reporting_command_(false);
    } else if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ENABLE) {
      this->send_enable_reporting_command_(true);
    } else if (this->pending_settings_command_ == 0x07) {
      this->send_installation_parameters_();
    } else if (this->pending_settings_command_ == 0x11) {
      this->send_detection_range_();
    } else if (this->pending_settings_command_ == 0x13) {
      this->send_sensitivity_command_(this->pending_sensitivity_);
    } else {
      this->send_installation_mode_command_(this->pending_installation_mode_);
    }
    this->last_command_ms_ = now;
    return;
  }

  if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ENABLE) {
    ESP_LOGE(TAG, "No acknowledgement while restoring LD2460 reporting; ending settings transaction.");
    this->settings_command_state_ = SettingsCommandState::IDLE;
    this->settings_retry_count_ = 0;
    return;
  }

  ESP_LOGW(TAG, "LD2460 settings command 0x%02X timed out.", this->pending_settings_command_);
  this->finish_settings_transaction_(false);
}

void LD2460Component::send_protocol_command_(const std::vector<uint8_t> &command) {
  this->write_array(command.data(), command.size());
  this->flush();
}

void LD2460Component::apply_installation_mode_(uint8_t mode) {
  if (mode != this->installation_mode_) {
    this->installation_parameters_response_received_ = false;
    this->detection_range_response_received_ = false;
    this->sensitivity_response_received_ = false;
    this->health_probe_active_ = false;  // A probe discovering a changed mode must read its settings too.
    this->presence_tracker_.clear();
    this->invalidate_tracking_states_();
  }
  this->installation_mode_ = mode;
  this->update_radar_boundary_();
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
  while (Protocol::take_frame(this->rx_buffer_, this->frame_buffer_, this->max_buffer_size_)) {
    if (Protocol::report_header(this->frame_buffer_))
      this->process_report_frame_(this->frame_buffer_);
    else
      this->process_command_frame_(this->frame_buffer_);
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

  if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_ENABLE &&
      this->restore_reporting_after_metadata_) {
    // A live report is stronger evidence than a lost command acknowledgement.
    this->reporting_enabled_ = true;
    this->startup_command_state_ = StartupCommandState::COMPLETE;
    this->startup_retry_count_ = 0;
  }

  Target targets[MAX_TARGETS]{};
  for (uint8_t i = 0; i < target_count; i++) {
    const size_t offset = 7 + i * 4;
    targets[i].raw_x = read_i16_le_(frame, offset);
    targets[i].raw_y = read_i16_le_(frame, offset + 2);
  }

  const uint32_t now = millis();
  this->note_radar_response_();
  // Occupancy uses every valid radar report. Network publication of coordinates
  // is intentionally slower and must never delay a presence transition.
  this->update_presence_tracks_(targets, target_count, now);
  bool coordinates_configured = false;
  for (const auto &sensors : this->target_sensors_)
    coordinates_configured |= sensors.x != nullptr || sensors.y != nullptr || sensors.distance != nullptr || sensors.angle != nullptr;
  if (!coordinates_configured)
    return;
  const bool should_publish = this->target_state_changed_(targets, target_count) &&
                              now - this->last_publish_ms_ >= this->publish_interval_ms_;

  // Coordinate conversion and trigonometry are only
  // useful for a log or state publish. Most reports need only the raw values
  // above to determine whether state changed.
  if (!should_publish) {
    return;
  }

  for (uint8_t i = 0; i < target_count; i++) {
    const float x_m = targets[i].raw_x / 10.0f;
    const float y_m = targets[i].raw_y / 10.0f;
    const float distance_m = std::sqrt(x_m * x_m + y_m * y_m);
    const float angle_deg = std::atan2(x_m, y_m) * RAD_TO_DEG;

    targets[i].x_m = x_m;
    targets[i].y_m = y_m;
    targets[i].distance_m = distance_m;
    targets[i].angle_deg = angle_deg;

  }

  if (should_publish) {
    this->publish_targets_(targets, target_count);
    this->last_publish_ms_ = now;
  }
}

void LD2460Component::process_command_frame_(const std::vector<uint8_t> &frame) {
  const uint8_t function_code = frame[4];
  const uint16_t frame_length = read_u16_le_(frame, 5);
  const size_t payload_length = frame_length - 11;
  const size_t payload_offset = 7;

  if (!Protocol::response_value(frame)) {
    ESP_LOGW(TAG, "Rejected malformed LD2460 reply 0x%02X.", function_code);
    return;
  }
  switch (function_code) {
    case 0x06: {
      if (payload_length < 1)
        break;
      const uint8_t result = frame[payload_offset];
      this->note_radar_response_();
      const bool success = (result & 0x10) != 0;
      const bool enabled = (result & 0x01) != 0;
      const bool temporary_startup_disable =
          this->startup_command_state_ == StartupCommandState::WAITING_FOR_DISABLE && !enabled;
      const bool temporary_settings_disable =
          this->settings_command_state_ == SettingsCommandState::WAITING_FOR_DISABLE && !enabled;
      const bool temporary_restore =
          (this->startup_command_state_ == StartupCommandState::WAITING_FOR_ENABLE &&
           enabled == this->restore_reporting_after_metadata_) ||
          (enabled && this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ENABLE);
      ESP_LOGI(TAG, "LD2460 reporting %s: %s", enabled ? "enable" : "disable", success ? "success" : "failed");
      if (success) {
        this->note_radar_response_();
        this->reporting_enabled_ = enabled;
        // A settings/metadata pause must not change the user-facing switch.
        // It represents the requested reporting state, not a brief transport
        // state used to make a command transaction reliable.
        if (this->reporting_switch_ != nullptr && !temporary_startup_disable && !temporary_settings_disable &&
            !temporary_restore)
          this->reporting_switch_->publish_state(enabled);

        if (!enabled && !temporary_startup_disable && !temporary_settings_disable)
          this->clear_tracking_states_();

        if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_DISABLE && !enabled) {
          this->startup_command_state_ = StartupCommandState::WAITING_FOR_METADATA;
          this->startup_queries_sent_ = false;
          this->startup_query_index_ = 0;
          this->startup_retry_count_ = 0;
          this->last_command_ms_ = millis();
        } else if (this->startup_command_state_ == StartupCommandState::WAITING_FOR_ENABLE &&
                   enabled == this->restore_reporting_after_metadata_) {
          this->startup_command_state_ = StartupCommandState::COMPLETE;
          this->startup_retry_count_ = 0;
        } else if (temporary_settings_disable) {
          this->settings_command_state_ = SettingsCommandState::WAITING_FOR_ACK;
          this->settings_retry_count_ = 0;
          if (this->pending_settings_command_ == 0x07)
            this->send_installation_parameters_();
          else if (this->pending_settings_command_ == 0x11)
            this->send_detection_range_();
          else if (this->pending_settings_command_ == 0x13)
            this->send_sensitivity_command_(this->pending_sensitivity_);
          else
            this->send_installation_mode_command_(this->pending_installation_mode_);
          this->last_command_ms_ = millis();
        } else if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ENABLE && enabled) {
          this->settings_command_state_ = SettingsCommandState::IDLE;
          this->settings_retry_count_ = 0;
        }
      }
      break;
    }
    case 0x0A: {
      if (payload_length < 1)
        break;
      if (frame[payload_offset] != 1 && frame[payload_offset] != 2)
        break;
      this->note_radar_response_();
      const char *mode = installation_mode_to_string_(frame[payload_offset]);
      this->apply_installation_mode_(frame[payload_offset]);
      ESP_LOGI(TAG, "LD2460 installation mode: %s", mode);
      if (this->installation_mode_text_sensor_ != nullptr &&
          (!this->installation_mode_text_sensor_->has_state() || this->installation_mode_text_sensor_->state != mode))
        this->installation_mode_text_sensor_->publish_state(mode);
      this->installation_mode_response_received_ = true;
      this->update_angle_limits_();
      this->advance_startup_query_(function_code);
      this->finish_metadata_queries_();
      break;
    }
    case 0x08: {
      if (payload_length < 4)
        break;
      if (this->installation_mode_ != 1)
        break;
      this->note_radar_response_();
      this->installation_height_m_ = read_u16_le_(frame, payload_offset) / 100.0f;
      this->installation_angle_deg_ = read_u16_le_(frame, payload_offset + 2) / 100.0f;
      this->installation_parameters_response_received_ = true;
      this->configuration_loaded_ = this->detection_range_response_received_;
      this->publish_config_values_();
      this->advance_startup_query_(function_code);
      this->finish_metadata_queries_();
      break;
    }
    case 0x12: {
      if (payload_length < 5)
        break;
      const uint8_t distance = frame[payload_offset];
      const int16_t start = read_i16_le_(frame, payload_offset + 1);
      const int16_t end = read_i16_le_(frame, payload_offset + 3);
      const bool top = this->installation_mode_ == 2;
      if (distance > (top ? 40 : 60) || start < (top ? 0 : -600) || end > (top ? 3600 : 600) || start >= end) {
        ESP_LOGW(TAG, "Rejected detection range outside the documented mounting-mode limits.");
        break;
      }
      this->note_radar_response_();
      this->detection_distance_m_ = distance / 10.0f;
      this->detection_start_angle_deg_ = start / 10.0f;
      this->detection_end_angle_deg_ = end / 10.0f;
      this->detection_range_response_received_ = true;
      this->update_radar_boundary_();
      this->configuration_loaded_ = this->installation_parameters_response_received_;
      this->publish_config_values_();
      this->advance_startup_query_(function_code);
      this->finish_metadata_queries_();
      break;
    }
    case 0x07:
    case 0x11: {
      this->note_radar_response_();
      if (this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ACK &&
          this->pending_settings_command_ == function_code)
        this->finish_settings_transaction_(frame[payload_offset] == 0x01);
      break;
    }
    case 0x09: {
      this->note_radar_response_();
      const uint8_t result = frame[payload_offset];
      // Low nibble echoes the requested mode; a stale ACK must not finish a different write.
      if (this->settings_command_state_ != SettingsCommandState::WAITING_FOR_ACK ||
          this->pending_settings_command_ != 0x09 || (result & 0x0F) != this->pending_installation_mode_)
        break;
      const bool success = (result & 0x10) != 0;
      if (success) {
        this->apply_installation_mode_(result & 0x0F);
        if (this->installation_mode_select_ != nullptr)
          this->installation_mode_select_->publish_state(this->installation_mode_ == 2 ? "Top" : "Side");
        if (this->installation_mode_text_sensor_ != nullptr)
          this->installation_mode_text_sensor_->publish_state(installation_mode_to_string_(this->installation_mode_));
        this->update_angle_limits_();
      }
      this->finish_settings_transaction_(success);
      break;
    }
    case 0x13: {
      this->note_radar_response_();
      if (payload_length >= 1 && this->settings_command_state_ == SettingsCommandState::WAITING_FOR_ACK &&
          this->pending_settings_command_ == 0x13) {
        const bool success = frame[payload_offset] == 0x01;
        if (success && this->sensitivity_select_ != nullptr) {
          this->sensitivity_select_->publish_state(this->pending_sensitivity_ == 1 ? "High" :
                                                    this->pending_sensitivity_ == 2 ? "Medium" : "Low");
        }
        this->finish_settings_transaction_(success);
      }
      break;
    }
    case 0x14: {
      if (payload_length < 1)
        break;
      this->note_radar_response_();
      const uint8_t sensitivity = frame[payload_offset];
      if (sensitivity >= 1 && sensitivity <= 3 && this->sensitivity_select_ != nullptr)
        this->sensitivity_select_->publish_state(sensitivity == 1 ? "High" : sensitivity == 2 ? "Medium" : "Low");
      this->sensitivity_response_received_ = true;
      this->advance_startup_query_(function_code);
      this->finish_metadata_queries_();
      break;
    }
    case 0x0B: {
      if (payload_length < 5)
        break;
      if (frame[payload_offset] != 1 && frame[payload_offset] != 2)
        break;
      this->note_radar_response_();
      const char *mode = installation_mode_to_string_(frame[payload_offset]);
      this->apply_installation_mode_(frame[payload_offset]);
      const uint8_t year = frame[payload_offset + 1];
      const uint8_t month = frame[payload_offset + 2];
      const uint8_t major = frame[payload_offset + 3];
      const uint8_t minor = frame[payload_offset + 4];

      char firmware[48];
      std::snprintf(firmware, sizeof(firmware), "%s V%u.%u (20%02u-%02u)", mode, major, minor, year, month);
      ESP_LOGI(TAG, "LD2460 firmware: %s", firmware);
      if (this->firmware_text_sensor_ != nullptr &&
          (!this->firmware_text_sensor_->has_state() || this->firmware_text_sensor_->state != firmware))
        this->firmware_text_sensor_->publish_state(firmware);
      if (this->installation_mode_text_sensor_ != nullptr &&
          (!this->installation_mode_text_sensor_->has_state() || this->installation_mode_text_sensor_->state != mode))
        this->installation_mode_text_sensor_->publish_state(mode);
      if (this->installation_mode_select_ != nullptr &&
          (!this->installation_mode_select_->has_state() || this->installation_mode_select_->current_option() !=
           (this->installation_mode_ == 2 ? "Top" : "Side")))
        this->installation_mode_select_->publish_state(this->installation_mode_ == 2 ? "Top" : "Side");
      this->firmware_response_received_ = true;
      this->advance_startup_query_(function_code);
      this->finish_metadata_queries_();
      break;
    }
    default:
      ESP_LOGI(TAG, "LD2460 command/ack function=0x%02X payload=%u byte(s): %s", function_code,
               static_cast<unsigned>(payload_length), format_frame_(frame).c_str());
      break;
  }

}

void LD2460Component::publish_presence_(bool present) {
  if (this->presence_state_known_ && this->presence_state_ == present)
    return;
  this->presence_state_known_ = true;
  this->presence_state_ = present;
  if (this->presence_binary_sensor_ != nullptr)
    this->presence_binary_sensor_->publish_state(present);
}

void LD2460Component::publish_tracking_state_() {
  const uint8_t count = this->presence_tracker_.get_count();
  this->publish_presence_(count != 0);
  if (!this->presence_count_known_ || count != this->last_published_presence_count_) {
    this->presence_count_known_ = true;
    this->last_published_presence_count_ = count;
    if (this->target_count_sensor_ != nullptr)
      this->target_count_sensor_->publish_state(count);
  }
}

void LD2460Component::invalidate_tracking_states_() {
  if (this->presence_state_known_ && this->presence_binary_sensor_ != nullptr)
    this->presence_binary_sensor_->invalidate_state();
  if (this->presence_count_known_ && this->target_count_sensor_ != nullptr)
    this->target_count_sensor_->publish_state(NAN);
  this->presence_state_known_ = false;
  this->presence_count_known_ = false;
}

void LD2460Component::update_presence_tracks_(const Target *targets, uint8_t target_count, uint32_t now) {
  PresenceTracker::Point points[MAX_TARGETS]{};
  for (uint8_t i = 0; i < target_count; i++) {
    points[i].x = targets[i].raw_x;
    points[i].y = targets[i].raw_y;
  }
  if (this->requested_reporting_enabled_) {
    this->presence_tracker_.update(points, target_count, now);
    this->publish_tracking_state_();
  }
}

bool LD2460Component::target_moved_for_publish_(const Target &current, const Target &previous) const {
  const int64_t threshold_sq = static_cast<int64_t>(this->position_update_threshold_dm_) *
                               this->position_update_threshold_dm_;
  const int32_t dx = static_cast<int32_t>(current.raw_x) - previous.raw_x;
  const int32_t dy = static_cast<int32_t>(current.raw_y) - previous.raw_y;
  return static_cast<int64_t>(dx) * dx + static_cast<int64_t>(dy) * dy >= threshold_sq;
}

void LD2460Component::publish_targets_(const Target *targets, uint8_t target_count) {
  const bool count_changed = !this->has_published_targets_ || target_count != this->last_published_target_count_;
  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    const bool present = i < target_count;
    const bool was_present = this->has_published_targets_ && i < this->last_published_target_count_;
    const bool moved = present &&
                       (count_changed || !was_present ||
                        this->target_moved_for_publish_(targets[i], this->last_published_targets_[i]));
    if (!moved && !(was_present && !present))
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
    this->last_published_targets_[i] = present ? targets[i] : Target{};
  }
  this->last_published_target_count_ = target_count;
  this->has_published_targets_ = true;
}

void LD2460Component::clear_tracking_states_() {
  this->invalidate_tracking_states_();
  for (uint8_t i = 0; i < this->last_published_target_count_; i++) {
    const auto target = this->target_sensors_[i];
    if (target.x != nullptr)
      target.x->publish_state(NAN);
    if (target.y != nullptr)
      target.y->publish_state(NAN);
    if (target.distance != nullptr)
      target.distance->publish_state(NAN);
    if (target.angle != nullptr)
      target.angle->publish_state(NAN);
  }
  this->presence_tracker_.clear();
  for (auto &target : this->last_published_targets_)
    target = Target{};
  this->last_published_target_count_ = 0;
  this->has_published_targets_ = true;
}

bool LD2460Component::target_state_changed_(const Target *targets, uint8_t target_count) const {
  if (!this->has_published_targets_ || target_count != this->last_published_target_count_)
    return true;
  for (uint8_t i = 0; i < target_count; i++) {
    if (this->target_moved_for_publish_(targets[i], this->last_published_targets_[i]))
      return true;
  }
  return false;
}

void LD2460Component::flush_unparsed_buffer_() {
  if (this->rx_buffer_.empty())
    return;

  const std::string frame = this->format_frame_(this->rx_buffer_);
  ESP_LOGW(TAG, "Unparsed RX %u byte(s): %s", static_cast<unsigned>(this->rx_buffer_.size()), frame.c_str());


  this->rx_buffer_.clear();
}

uint16_t LD2460Component::read_u16_le_(const std::vector<uint8_t> &bytes, size_t index) {
  return Protocol::u16(bytes, index);
}

int16_t LD2460Component::read_i16_le_(const std::vector<uint8_t> &bytes, size_t index) {
  return Protocol::i16(bytes, index);
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
