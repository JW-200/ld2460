#pragma once

#include <cstdint>

namespace esphome {
namespace ld2460 {

// Tracks up to five anonymous radar positions without allocating memory.
// Target slots are not person IDs, so each report is matched by proximity.
class PresenceTracker {
 public:
  static const uint8_t MAX_TRACKS = 5;
  struct Point {
    int16_t x{0};  // decimetres
    int16_t y{0};
  };

  void set_presence_timeout(uint32_t value) { this->presence_timeout_ms_ = value; }
  void set_stationary_presence_timeout(uint32_t value) { this->stationary_presence_timeout_ms_ = value; }
  void set_stationary_dwell(uint32_t value) { this->stationary_dwell_ms_ = value; }
  uint32_t get_presence_timeout() const { return this->presence_timeout_ms_; }
  uint32_t get_stationary_presence_timeout() const { return this->stationary_presence_timeout_ms_; }
  uint32_t get_stationary_dwell() const { return this->stationary_dwell_ms_; }

  void clear() {
    for (auto &track : this->tracks_)
      track = Track{};
  }

  bool update(const Point *points, uint8_t count, uint32_t now) {
    if (count > MAX_TRACKS)
      count = MAX_TRACKS;
    bool point_matched[MAX_TRACKS]{};
    bool track_matched[MAX_TRACKS]{};

    // Use a global nearest-pair search so a change in radar slot order does
    // not immediately create a new person. Five tracks bound the work.
    for (uint8_t pair = 0; pair < count; pair++) {
      uint8_t best_point = MAX_TRACKS;
      uint8_t best_track = MAX_TRACKS;
      int64_t best_distance_sq = MATCH_RADIUS_SQ + 1;
      for (uint8_t point_index = 0; point_index < count; point_index++) {
        if (point_matched[point_index])
          continue;
        for (uint8_t track_index = 0; track_index < MAX_TRACKS; track_index++) {
          const auto &track = this->tracks_[track_index];
          if (!track.active || track_matched[track_index] ||
              now - track.last_seen_ms >= this->timeout_for_(track))
            continue;
          const int64_t distance_sq = squared_distance_(points[point_index], track.position);
          if (distance_sq <= MATCH_RADIUS_SQ && distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_point = point_index;
            best_track = track_index;
          }
        }
      }
      if (best_track == MAX_TRACKS)
        break;

      auto &track = this->tracks_[best_track];
      const auto &point = points[best_point];
      if (squared_distance_(point, track.anchor) > STATIONARY_RADIUS_SQ) {
        // A single displaced radar report must not erase a long stationary
        // stay. Require sustained movement before starting a new dwell.
        if (!track.moving || now - track.last_seen_ms >= MOVING_DWELL_MS) {
          track.moving = true;
          track.moving_since_ms = now;
        } else if (now - track.moving_since_ms >= MOVING_DWELL_MS) {
          track.anchor = point;
          track.stationary_since_ms = now;
          track.stationary = false;
          track.moving = false;
        }
      } else {
        track.moving = false;
      }
      if (!track.moving && now - track.stationary_since_ms >= this->stationary_dwell_ms_) {
        track.stationary = true;
      }
      track.position = point;
      track.last_seen_ms = now;
      point_matched[best_point] = true;
      track_matched[best_track] = true;
    }

    // Give each unmatched detection a free track, or replace the oldest
    // absent track when all five slots are occupied.
    for (uint8_t point_index = 0; point_index < count; point_index++) {
      if (point_matched[point_index])
        continue;
      uint8_t selected = MAX_TRACKS;
      uint32_t oldest_age = 0;
      for (uint8_t track_index = 0; track_index < MAX_TRACKS; track_index++) {
        const auto &track = this->tracks_[track_index];
        if (track_matched[track_index])
          continue;
        if (!track.active) {
          selected = track_index;
          break;
        }
        const uint32_t age = now - track.last_seen_ms;
        if (selected == MAX_TRACKS || age > oldest_age) {
          selected = track_index;
          oldest_age = age;
        }
      }
      if (selected == MAX_TRACKS)
        continue;
      auto &track = this->tracks_[selected];
      track = Track{};
      track.position = points[point_index];
      track.anchor = points[point_index];
      track.last_seen_ms = now;
      track.stationary_since_ms = now;
      track.active = true;
      track_matched[selected] = true;
    }

    bool occupied = false;
    for (uint8_t i = 0; i < MAX_TRACKS; i++) {
      auto &track = this->tracks_[i];
      if (!track.active)
        continue;
      if (!track_matched[i] && now - track.last_seen_ms >= this->timeout_for_(track))
        track.active = false;
      occupied |= track.active;
    }
    return occupied;
  }

 private:
  struct Track {
    Point position{};
    Point anchor{};
    uint32_t last_seen_ms{0};
    uint32_t stationary_since_ms{0};
    uint32_t moving_since_ms{0};
    bool active{false};
    bool stationary{false};
    bool moving{false};
  };

  static const int64_t MATCH_RADIUS_SQ = 12 * 12;  // 1.2 m
  static const int64_t STATIONARY_RADIUS_SQ = 6 * 6;  // 0.6 m
  static const uint32_t MOVING_DWELL_MS = 5000;

  static int64_t squared_distance_(const Point &a, const Point &b) {
    const int32_t dx = static_cast<int32_t>(a.x) - b.x;
    const int32_t dy = static_cast<int32_t>(a.y) - b.y;
    return static_cast<int64_t>(dx) * dx + static_cast<int64_t>(dy) * dy;
  }

  uint32_t timeout_for_(const Track &track) const {
    return track.stationary ? this->stationary_presence_timeout_ms_ : this->presence_timeout_ms_;
  }

  Track tracks_[MAX_TRACKS]{};
  uint32_t presence_timeout_ms_{30000};
  uint32_t stationary_presence_timeout_ms_{1800000};
  uint32_t stationary_dwell_ms_{30000};
};

}  // namespace ld2460
}  // namespace esphome
