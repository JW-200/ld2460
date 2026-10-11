#pragma once

#include <cmath>
#include <cstdint>

namespace esphome {
namespace ld2460 {

// Coordinates and distances are in decimetres, like the radar reports.
struct TrackingPoint {
  int16_t x{0};
  int16_t y{0};
};

class TrackingGeometry {
 public:
  static float distance(TrackingPoint a, TrackingPoint b) {
    const float dx = static_cast<float>(a.x) - b.x;
    const float dy = static_cast<float>(a.y) - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }
  static float segment_distance(TrackingPoint p, TrackingPoint a, TrackingPoint b) {
    const float dx = static_cast<float>(b.x) - a.x;
    const float dy = static_cast<float>(b.y) - a.y;
    const float length_sq = dx * dx + dy * dy;
    if (length_sq == 0)
      return distance(p, a);
    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    const float px = p.x - (a.x + t * dx);
    const float py = p.y - (a.y + t * dy);
    return std::sqrt(px * px + py * py);
  }
  static bool valid_zone(const TrackingPoint *vertices) {
    int64_t area = 0;
    for (uint8_t i = 0; i < 4; i++) {
      const auto a = vertices[i];
      const auto b = vertices[(i + 1) % 4];
      if (a.x == b.x && a.y == b.y)
        return false;
      area += static_cast<int64_t>(a.x) * b.y - static_cast<int64_t>(a.y) * b.x;
    }
    return area != 0 && !intersects_(vertices[0], vertices[1], vertices[2], vertices[3]) &&
           !intersects_(vertices[1], vertices[2], vertices[3], vertices[0]);
  }
  static bool contains(const TrackingPoint *vertices, TrackingPoint p, float margin = 0) {
    bool inside = false;
    for (uint8_t i = 0, j = 3; i < 4; j = i++) {
      const auto a = vertices[j];
      const auto b = vertices[i];
      if (segment_distance(p, a, b) <= margin + 0.001f)
        return true;
      if ((a.y > p.y) != (b.y > p.y) &&
          p.x < a.x + static_cast<float>(b.x - a.x) * (p.y - a.y) / (b.y - a.y))
        inside = !inside;
    }
    return inside;
  }
 private:
  static int64_t cross_(TrackingPoint a, TrackingPoint b, TrackingPoint p) {
    return static_cast<int64_t>(b.x - a.x) * (p.y - a.y) -
           static_cast<int64_t>(b.y - a.y) * (p.x - a.x);
  }
  static bool intersects_(TrackingPoint a, TrackingPoint b, TrackingPoint c, TrackingPoint d) {
    const auto ab_c = cross_(a, b, c), ab_d = cross_(a, b, d);
    const auto cd_a = cross_(c, d, a), cd_b = cross_(c, d, b);
    // Opposite edges touching also make the room polygon invalid.
    if ((ab_c == 0 && segment_distance(c, a, b) < 0.001f) ||
        (ab_d == 0 && segment_distance(d, a, b) < 0.001f) ||
        (cd_a == 0 && segment_distance(a, c, d) < 0.001f) ||
        (cd_b == 0 && segment_distance(b, c, d) < 0.001f))
      return true;
    return ((ab_c > 0) != (ab_d > 0)) && ((cd_a > 0) != (cd_b > 0));
  }
};

}  // namespace ld2460
}  // namespace esphome
