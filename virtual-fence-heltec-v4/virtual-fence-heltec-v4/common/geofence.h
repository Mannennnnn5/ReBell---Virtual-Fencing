// =============================================================================
//  Geofence maths – point-in-polygon + signed distance to the fence line.
//  Pure C++, no Arduino dependency, so it can be unit-tested on a PC.
//
//  The polygon is projected to a local flat x/y plane in metres
//  (equirectangular around the first vertex). Error < 0.1 % for pastures a
//  few km across, which is far below GNSS noise.
// =============================================================================
#pragma once
#include <math.h>
#include <stdint.h>
#include "protocol.h"

struct Geofence {
  uint8_t      version = 0;
  uint8_t      count   = 0;                       // vertices
  VfFencePoint pts[VF_MAX_FENCE_POINTS];

  bool valid() const { return count >= 3; }
};

namespace geo {

constexpr double kEarthR  = 6371000.0;
constexpr double kDegRad  = M_PI / 180.0;

// project (lat,lon in 1e-7 deg) to metres relative to an origin
inline void toLocal(int32_t lat, int32_t lon, int32_t lat0, int32_t lon0,
                    double &x, double &y) {
  const double cosLat0 = cos(lat0 * 1e-7 * kDegRad);
  x = (lon - lon0) * 1e-7 * kDegRad * kEarthR * cosLat0;
  y = (lat - lat0) * 1e-7 * kDegRad * kEarthR;
}

// distance from point P to segment AB (all in metres)
inline double distToSegment(double px, double py, double ax, double ay,
                            double bx, double by) {
  const double dx = bx - ax, dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0;
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  const double cx = ax + t * dx - px, cy = ay + t * dy - py;
  return sqrt(cx * cx + cy * cy);
}

// Signed distance in metres: positive = inside the fence, negative = outside.
// Returns NAN if the fence is not valid.
inline double signedDistance(const Geofence &f, int32_t lat, int32_t lon) {
  if (!f.valid()) return NAN;
  const int32_t lat0 = f.pts[0].lat, lon0 = f.pts[0].lon;
  double px, py;
  toLocal(lat, lon, lat0, lon0, px, py);

  bool inside = false;
  double best = INFINITY;
  for (uint8_t i = 0, j = f.count - 1; i < f.count; j = i++) {
    double ax, ay, bx, by;
    toLocal(f.pts[i].lat, f.pts[i].lon, lat0, lon0, ax, ay);
    toLocal(f.pts[j].lat, f.pts[j].lon, lat0, lon0, bx, by);
    // ray casting (crossing number)
    if (((ay > py) != (by > py)) &&
        (px < (bx - ax) * (py - ay) / (by - ay) + ax))
      inside = !inside;
    const double d = distToSegment(px, py, ax, ay, bx, by);
    if (d < best) best = d;
  }
  return inside ? best : -best;
}

}  // namespace geo
