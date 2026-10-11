#pragma once

#include "tracking_geometry.h"

#include <cstdint>
#include <initializer_list>

namespace esphome {
namespace ld2460 {

// Anonymous tracks: radar slot order is not a persistent person identifier.
// Local, bounded state; positions need not be sent over the network.
class PresenceTracker {
 public:
  static constexpr uint8_t MAX_TRACKS = 5;
  using Point = TrackingPoint;

  void set_presence_timeout(uint32_t value) { this->presence_timeout_ms_ = value; }
  void set_stationary_presence_timeout(uint32_t value) { this->stationary_presence_timeout_ms_ = value; }
  void set_stationary_dwell(uint32_t value) { this->stationary_dwell_ms_ = value; }
  void set_exit_timeout(uint32_t value) { this->exit_timeout_ms_ = value; }
  void set_edge_margin(float metres) { this->edge_margin_dm_ = metres * 10; }
  uint32_t get_presence_timeout() const { return this->presence_timeout_ms_; }
  uint32_t get_stationary_presence_timeout() const { return this->stationary_presence_timeout_ms_; }
  uint32_t get_stationary_dwell() const { return this->stationary_dwell_ms_; }
  uint32_t get_exit_timeout() const { return this->exit_timeout_ms_; }

  bool set_zone(const Point *vertices, bool enabled) {
    if (enabled && !TrackingGeometry::valid_zone(vertices))
      return false;
    for (uint8_t i = 0; i < 4; i++)
      this->vertices_[i] = vertices[i];
    this->zone_enabled_ = enabled;
    this->clear_exit_evidence_();
    return true;
  }

  // 0 = disabled; 1..4 = P1-P2, P2-P3, P3-P4, P4-P1.
  void set_exit_edges(uint8_t first, uint8_t second) {
    this->exit_edges_[0] = first <= 4 ? first : 0;
    this->exit_edges_[1] = second <= 4 ? second : 0;
    this->clear_exit_evidence_();
  }

  void set_radar_boundary(float distance_m, float start_deg, float end_deg, bool enabled) {
    const bool previous_enabled = this->radar_boundary_enabled_;
    const float previous_distance = this->radar_distance_dm_;
    const Point previous_start = this->radar_start_, previous_end = this->radar_end_;
    this->radar_boundary_enabled_ = enabled && distance_m > 0 && start_deg < end_deg;
    this->radar_distance_dm_ = distance_m * 10;
    constexpr float DEG_TO_RAD = 0.01745329252f;
    this->radar_start_ = Point{
        static_cast<int16_t>(std::lround(this->radar_distance_dm_ * std::sin(start_deg * DEG_TO_RAD))),
        static_cast<int16_t>(std::lround(this->radar_distance_dm_ * std::cos(start_deg * DEG_TO_RAD)))};
    this->radar_end_ = Point{
        static_cast<int16_t>(std::lround(this->radar_distance_dm_ * std::sin(end_deg * DEG_TO_RAD))),
        static_cast<int16_t>(std::lround(this->radar_distance_dm_ * std::cos(end_deg * DEG_TO_RAD)))};
    if (previous_enabled != this->radar_boundary_enabled_ || previous_distance != this->radar_distance_dm_ ||
        previous_start.x != this->radar_start_.x || previous_start.y != this->radar_start_.y ||
        previous_end.x != this->radar_end_.x || previous_end.y != this->radar_end_.y)
      this->clear_exit_evidence_();
  }

  void clear() {
    for (auto &track : this->tracks_)
      track = Track{};
  }

  uint8_t get_count() const {
    uint8_t count = 0;
    for (const auto &track : this->tracks_)
      count += track.active && track.confirmed;
    return count;
  }

  bool age(uint32_t now) {
    for (auto &track : this->tracks_) {
      if (track.active && now - track.last_seen_ms >= this->timeout_for_(track))
        track = Track{};
    }
    return this->get_count() != 0;
  }

  bool update(const Point *points, uint8_t count, uint32_t now) {
    this->age(now);
    if (points == nullptr)
      count = 0;
    if (count > MAX_TRACKS)
      count = MAX_TRACKS;
    Point accepted[MAX_TRACKS]{};
    bool can_start[MAX_TRACKS]{};
    uint8_t accepted_count = 0;
    for (uint8_t i = 0; i < count; i++) {
      const bool inside = !this->zone_enabled_ || TrackingGeometry::contains(this->vertices_, points[i]);
      // A 20 cm border tolerance is for existing tracks only.
      if (!inside && !TrackingGeometry::contains(this->vertices_, points[i], 2.0f))
        continue;
      accepted[accepted_count] = points[i];
      can_start[accepted_count++] = inside;
    }

    // Maximum-cardinality, minimum-distance matching avoids the greedy
    // assignment stealing another person's only plausible match.
    Assignment assignment{};
    this->assign_(accepted, accepted_count, now, 0, 0, 0, 0, assignment);
    bool matched[MAX_TRACKS]{};
    for (uint8_t i = 0; i < accepted_count; i++) {
      const uint8_t index = assignment.best[i];
      if (index == MAX_TRACKS)
        continue;
      matched[index] = true;
      this->observe_(this->tracks_[index], accepted[i], now);
    }

    for (uint8_t i = 0; i < accepted_count; i++) {
      if (assignment.best[i] != MAX_TRACKS || !can_start[i])
        continue;
      uint8_t selected = MAX_TRACKS;
      uint32_t oldest_age = 0;
      for (uint8_t j = 0; j < MAX_TRACKS; j++) {
        if (matched[j])
          continue;
        const auto &track = this->tracks_[j];
        if (!track.active) {
          selected = j;
          break;
        }
        // Protect recently detected people when capacity is reached.
        const uint32_t age = now - track.last_seen_ms;
        if (age >= CONFIRM_TIMEOUT_MS && (selected == MAX_TRACKS || age > oldest_age)) {
          selected = j;
          oldest_age = age;
        }
      }
      if (selected == MAX_TRACKS)
        continue;
      auto &track = this->tracks_[selected];
      track = Track{};
      track.position = accepted[i];
      track.anchor = accepted[i];
      track.motion_reference = accepted[i];
      track.first_seen_ms = now;
      track.last_seen_ms = now;
      track.motion_reference_ms = now;
      track.active = true;
      matched[selected] = true;
    }
    return this->get_count() != 0;
  }

 private:
  struct Track {
    Point position{};
    Point anchor{};
    Point motion_reference{};
    uint32_t first_seen_ms{0};
    uint32_t last_seen_ms{0};
    uint32_t stationary_observed_ms{0};
    uint32_t movement_started_ms{0};
    uint32_t motion_reference_ms{0};
    uint32_t exit_evidence_ms{0};
    bool active{false};
    bool confirmed{false};
    bool stationary{false};
    bool movement_pending{false};
    bool exit_candidate{false};
  };
  struct Assignment {
    uint8_t current[MAX_TRACKS]{MAX_TRACKS, MAX_TRACKS, MAX_TRACKS, MAX_TRACKS, MAX_TRACKS};
    uint8_t best[MAX_TRACKS]{MAX_TRACKS, MAX_TRACKS, MAX_TRACKS, MAX_TRACKS, MAX_TRACKS};
    int best_matches{-1};
    float best_cost{0};
  };

  static constexpr uint32_t CONFIRM_TIMEOUT_MS = 1000;
  static constexpr uint32_t CONFIRM_DWELL_MS = 200;
  static constexpr uint32_t OBSERVATION_GAP_MS = 2000;
  static constexpr uint32_t MOVEMENT_DWELL_MS = 300;
  static constexpr uint32_t MOTION_WINDOW_MS = 1000;
  static constexpr uint32_t EXIT_EVIDENCE_MS = 3000;
  static constexpr float STATIONARY_RADIUS_DM = 6;
  static constexpr float DIRECTION_THRESHOLD_DM = 2;

  void assign_(const Point *points, uint8_t count, uint32_t now, uint8_t point_index, uint8_t used,
               int matches, float cost, Assignment &result) const {
    if (matches + count - point_index < result.best_matches)
      return;
    if (point_index == count) {
      if (matches > result.best_matches || (matches == result.best_matches && cost < result.best_cost)) {
        result.best_matches = matches;
        result.best_cost = cost;
        for (uint8_t i = 0; i < count; i++)
          result.best[i] = result.current[i];
      }
      return;
    }
    for (uint8_t j = 0; j < MAX_TRACKS; j++) {
      const auto &track = this->tracks_[j];
      if (!track.active || (used & (1U << j)) != 0)
        continue;
      const uint32_t gap = now - track.last_seen_ms;
      // Walking during a short dropout permits a wider match. Old seated
      // tracks remain bounded so they do not absorb a new arrival.
      const float gate = gap <= OBSERVATION_GAP_MS ? 12.0f + 18.0f * gap / OBSERVATION_GAP_MS : 12.0f;
      const float distance = TrackingGeometry::distance(points[point_index], track.position);
      if (distance > gate)
        continue;
      result.current[point_index] = j;
      this->assign_(points, count, now, point_index + 1, used | (1U << j), matches + 1,
                    cost + distance * distance, result);
    }
    result.current[point_index] = MAX_TRACKS;
    this->assign_(points, count, now, point_index + 1, used, matches, cost, result);
  }

  void observe_(Track &track, Point point, uint32_t now) {
    const uint32_t gap = now - track.last_seen_ms;
    if (!track.confirmed && now - track.first_seen_ms >= CONFIRM_DWELL_MS)
      track.confirmed = true;

    if (TrackingGeometry::distance(point, track.anchor) > STATIONARY_RADIUS_DM) {
      if (!track.movement_pending || gap > OBSERVATION_GAP_MS) {
        track.movement_pending = true;
        track.movement_started_ms = now;
      } else if (now - track.movement_started_ms >= MOVEMENT_DWELL_MS) {
        // Two separated displaced observations suffice; no five-second walk
        // is required to leave the stationary hold.
        track.stationary = false;
        track.stationary_observed_ms = 0;
        track.anchor = point;
        track.movement_pending = false;
      }
    } else {
      track.movement_pending = false;
      if (gap <= OBSERVATION_GAP_MS) {
        const uint32_t accumulated = track.stationary_observed_ms < this->stationary_dwell_ms_
                                         ? track.stationary_observed_ms : this->stationary_dwell_ms_;
        const uint32_t remaining = this->stationary_dwell_ms_ - accumulated;
        track.stationary_observed_ms = accumulated + (gap < remaining ? gap : remaining);
      } else if (!track.stationary) {
        track.stationary_observed_ms = 0;
      }
      if (track.stationary_observed_ms >= this->stationary_dwell_ms_)
        track.stationary = true;
    }

    const uint32_t motion_age = now - track.motion_reference_ms;
    if (motion_age > OBSERVATION_GAP_MS || gap > OBSERVATION_GAP_MS) {
      track.exit_candidate = false;
      track.motion_reference = point;
      track.motion_reference_ms = now;
    } else {
      // A reversal is visible in the latest step even when the one-second
      // reference still lies further inside the room.
      if (track.exit_candidate && TrackingGeometry::distance(point, track.position) >= DIRECTION_THRESHOLD_DM &&
          !this->approaching_boundary_(track.position, point))
        track.exit_candidate = false;
      if (TrackingGeometry::distance(point, track.motion_reference) >= DIRECTION_THRESHOLD_DM) {
        const bool approaching = this->approaching_boundary_(track.motion_reference, point);
        track.exit_candidate = approaching;
        if (approaching)
          track.exit_evidence_ms = now;
      } else if (track.exit_candidate && now - track.exit_evidence_ms > EXIT_EVIDENCE_MS) {
        track.exit_candidate = false;
      }
      if (motion_age >= MOTION_WINDOW_MS) {
        track.motion_reference = point;
        track.motion_reference_ms = now;
      }
    }
    track.position = point;
    track.last_seen_ms = now;
  }

  bool approaching_boundary_(Point previous, Point current) const {
    if (this->zone_enabled_) {
      for (const auto edge : this->exit_edges_) {
        if (edge == 0)
          continue;
        const auto a = this->vertices_[edge - 1], b = this->vertices_[edge % 4];
        const float before = TrackingGeometry::segment_distance(previous, a, b);
        const float after = TrackingGeometry::segment_distance(current, a, b);
        if (after <= this->edge_margin_dm_ && before - after >= DIRECTION_THRESHOLD_DM)
          return true;
      }
    }
    if (this->radar_boundary_enabled_) {
      const float before = this->radar_distance_dm_ - TrackingGeometry::distance(previous, Point{});
      const float after = this->radar_distance_dm_ - TrackingGeometry::distance(current, Point{});
      if (after <= this->edge_margin_dm_ && before - after >= DIRECTION_THRESHOLD_DM)
        return true;
      for (const auto end : {this->radar_start_, this->radar_end_}) {
        const float ray_before = TrackingGeometry::segment_distance(previous, Point{}, end);
        const float ray_after = TrackingGeometry::segment_distance(current, Point{}, end);
        if (ray_after <= this->edge_margin_dm_ && ray_before - ray_after >= DIRECTION_THRESHOLD_DM)
          return true;
      }
    }
    return false;
  }

  uint32_t timeout_for_(const Track &track) const {
    if (!track.confirmed)
      return CONFIRM_TIMEOUT_MS;
    const uint32_t normal = track.stationary ? this->stationary_presence_timeout_ms_ : this->presence_timeout_ms_;
    return track.exit_candidate && this->exit_timeout_ms_ < normal ? this->exit_timeout_ms_ : normal;
  }
  void clear_exit_evidence_() {
    for (auto &track : this->tracks_)
      track.exit_candidate = false;
  }

  Track tracks_[MAX_TRACKS]{};
  Point vertices_[4]{};
  Point radar_start_{};
  Point radar_end_{};
  uint8_t exit_edges_[2]{};
  uint32_t presence_timeout_ms_{20000};
  uint32_t stationary_presence_timeout_ms_{3600000};
  uint32_t stationary_dwell_ms_{10000};
  uint32_t exit_timeout_ms_{5000};
  float edge_margin_dm_{5};
  float radar_distance_dm_{60};
  bool zone_enabled_{false};
  bool radar_boundary_enabled_{false};
};

}  // namespace ld2460
}  // namespace esphome
