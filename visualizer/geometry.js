/* X directions are from a person facing the radar: +X their right, -X their left.
 * In a top-down plan with +Y up, that person's +X points to page-left.
 * Heading turns the radar's +Y clockwise from page-up. */
(function (root) {
  "use strict";
  const EPS = 1e-8;
  function rotate(point, heading) {
    const a = heading * Math.PI / 180;
    return { x: -point.x * Math.cos(a) + point.y * Math.sin(a),
      y: point.x * Math.sin(a) + point.y * Math.cos(a) };
  }
  function unrotate(point, heading) {
    const a = heading * Math.PI / 180;
    // Undo the heading, then undo the facing-radar X reflection.
    return { x: -point.x * Math.cos(a) + point.y * Math.sin(a),
      y: point.x * Math.sin(a) + point.y * Math.cos(a) };
  }
  function cross(a, b, p) { return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x); }
  function onSegment(p, a, b) {
    return Math.abs(cross(a, b, p)) < EPS && p.x >= Math.min(a.x, b.x) - EPS &&
      p.x <= Math.max(a.x, b.x) + EPS && p.y >= Math.min(a.y, b.y) - EPS && p.y <= Math.max(a.y, b.y) + EPS;
  }
  function intersects(a, b, c, d) {
    if (onSegment(c, a, b) || onSegment(d, a, b) || onSegment(a, c, d) || onSegment(b, c, d)) return true;
    return (cross(a, b, c) > 0) !== (cross(a, b, d) > 0) &&
      (cross(c, d, a) > 0) !== (cross(c, d, b) > 0);
  }
  function area(points) {
    return Math.abs(points.reduce((sum, a, i) => {
      const b = points[(i + 1) % points.length];
      return sum + a.x * b.y - a.y * b.x;
    }, 0)) / 2;
  }
  function validate(points) {
    if (points.length !== 4 || points.some(p => !Number.isFinite(p.x) || !Number.isFinite(p.y)))
      return "Enter all eight coordinates.";
    if (points.some(p => Math.abs(p.x) > 12 || Math.abs(p.y) > 12))
      return "Coordinates must be between −12 and +12 m.";
    if (points.some((a, i) => points.some((b, j) => i !== j && a.x === b.x && a.y === b.y)))
      return "Corners overlap. Give each corner a different position.";
    if (intersects(points[0], points[1], points[2], points[3]) ||
        intersects(points[1], points[2], points[3], points[0]))
      return "Edges cross or touch. Enter P1 → P2 → P3 → P4 around the perimeter.";
    if (area(points) < EPS) return "The room has no area. Move the corners apart.";
    return "";
  }
  function contains(points, p) {
    let inside = false;
    for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
      const a = points[j], b = points[i];
      if (onSegment(p, a, b)) return true;
      if ((a.y > p.y) !== (b.y > p.y) && p.x < a.x + (b.x - a.x) * (p.y - a.y) / (b.y - a.y)) inside = !inside;
    }
    return inside;
  }
  function snap(value) { return Math.max(-12, Math.min(12, Math.round(value * 10) / 10)) || 0; }
  const api = { rotate, unrotate, area, validate, contains, snap };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.RadarGeometry = api;
})(typeof globalThis !== "undefined" ? globalThis : this);
