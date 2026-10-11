#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace esphome {
namespace ld2460 {

// HLK-LD2460 Serial port communication protocol V1.0, tables 1-28.
class Protocol {
 public:
  static uint16_t u16(const std::vector<uint8_t> &bytes, size_t offset) {
    return uint16_t(bytes[offset]) | (uint16_t(bytes[offset + 1]) << 8);
  }
  static int16_t i16(const std::vector<uint8_t> &bytes, size_t offset) {
    const uint16_t raw = u16(bytes, offset);
    return static_cast<int16_t>(raw < 0x8000 ? int32_t(raw) : int32_t(raw) - 0x10000);
  }
  static bool report_header(const std::vector<uint8_t> &bytes) {
    return bytes.size() >= 4 && bytes[0] == 0xF4 && bytes[1] == 0xF3 && bytes[2] == 0xF2 && bytes[3] == 0xF1;
  }
  static bool command_header(const std::vector<uint8_t> &bytes) {
    return bytes.size() >= 4 && bytes[0] == 0xFD && bytes[1] == 0xFC && bytes[2] == 0xFB && bytes[3] == 0xFA;
  }
  static bool response_length(uint8_t function, size_t length) {
    switch (function) {
      case 0x06: case 0x07: case 0x09: case 0x0A: case 0x11: case 0x13: case 0x14: return length == 12;
      case 0x08: return length == 15;
      case 0x0B: case 0x12: return length == 16;
      default: return length >= 11;  // Unimplemented functions may be logged, never treated as health replies.
    }
  }
  static bool response_value(const std::vector<uint8_t> &frame) {
    if (frame.size() < 11 || !response_length(frame[4], frame.size()))
      return false;
    const uint8_t value = frame.size() > 11 ? frame[7] : 0;
    switch (frame[4]) {
      case 0x06: return value == 0x00 || value == 0x01 || value == 0x10 || value == 0x11;
      case 0x09: return value == 0x01 || value == 0x02 || value == 0x11 || value == 0x12;
      case 0x07: case 0x13: case 0x11: return value <= 1;
      case 0x0A: case 0x0B: return value == 1 || value == 2;
      case 0x14: return value >= 1 && value <= 3;
      default: return true;
    }
  }
  // Extract a complete frame, retaining partial headers/frames across UART reads.
  // A bad footer drops ONE byte so a following valid frame is not swallowed.
  static bool take_frame(std::vector<uint8_t> &rx, std::vector<uint8_t> &frame, size_t maximum) {
    while (rx.size() >= 4) {
      const bool report = report_header(rx);
      if (!report && !command_header(rx)) {
        rx.erase(rx.begin());
        continue;
      }
      if (rx.size() < 7)
        return false;
      const size_t length = u16(rx, 5);
      const bool shape = report ? rx[4] == 0x04 && length >= 11 && length <= 31 && (length - 11) % 4 == 0
                                : response_length(rx[4], length);
      if (!shape || length < 11 || length > maximum) {
        rx.erase(rx.begin());
        continue;
      }
      if (rx.size() < length)
        return false;
      const size_t tail = length - 4;
      const uint8_t first = report ? 0xF8 : 0x04;
      if (rx[tail] != first || rx[tail + 1] != first - 1 || rx[tail + 2] != first - 2 ||
          rx[tail + 3] != first - 3) {
        rx.erase(rx.begin());
        continue;
      }
      frame.assign(rx.begin(), rx.begin() + length);
      rx.erase(rx.begin(), rx.begin() + length);
      return true;
    }
    return false;
  }
  static std::vector<uint8_t> command(uint8_t function, std::initializer_list<uint8_t> payload) {
    const size_t length = 11 + payload.size();
    std::vector<uint8_t> bytes{0xFD, 0xFC, 0xFB, 0xFA, function, uint8_t(length), uint8_t(length >> 8)};
    bytes.insert(bytes.end(), payload);
    bytes.insert(bytes.end(), {0x04, 0x03, 0x02, 0x01});
    return bytes;
  }
  static std::vector<uint8_t> installation(uint16_t height_cm, uint16_t angle_centidegrees) {
    return command(0x07, {uint8_t(height_cm), uint8_t(height_cm >> 8),
                          uint8_t(angle_centidegrees), uint8_t(angle_centidegrees >> 8)});
  }
  static std::vector<uint8_t> range(uint8_t distance_dm, int16_t start_decidegrees, int16_t end_decidegrees) {
    const auto start = static_cast<uint16_t>(start_decidegrees);
    const auto end = static_cast<uint16_t>(end_decidegrees);
    return command(0x11, {distance_dm, uint8_t(start), uint8_t(start >> 8), uint8_t(end), uint8_t(end >> 8)});
  }
};

}  // namespace ld2460
}  // namespace esphome
