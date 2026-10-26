// filter_vr_overlay_math.hpp - per-pixel math of the §4 placement layer, ported line by line from
// vredit/place/layer.py (PlacedLayer.resolve, render_layer_eye) and vredit/place/surfaces.py (Quad, Cylinder,
// SpherePatch). Same conventions as vrcore.hpp: OpenXR axes, metres, degrees at the API, pixel centres at +0.5.
// GPL-2.0-or-later.
#pragma once
#include "vrcore.hpp"
#include <cmath>
#include <limits>

namespace vrov {
using vrcore::DEG;
using vrcore::Mat3;
using vrcore::Vec3;

constexpr double EPS = 1e-12;
constexpr double INF = std::numeric_limits<double>::infinity();

enum class Shape { Flat, Cylinder, Sphere };
enum class Rig { None, Parallel, ODS };

// ----------------------------------------------------------------- stereo.py: ray origins
// eye 0 = L, 1 = R; eye_sign L = -1, R = +1.
inline Vec3 ray_origin(Rig rig, double baseline, int eye, Vec3 d)
{
    double s = eye == 0 ? -1.0 : 1.0;
    if (rig == Rig::Parallel) return {s * baseline / 2, 0, 0};
    if (rig == Rig::ODS) {  // ODSRig without pole merge: tangent to the circle of radius baseline/2
        double lon, lat;
        vrcore::dir_to_lonlat(d, lon, lat);
        double r = s * baseline / 2;
        return {r * std::cos(lon), 0, r * std::sin(lon)};
    }
    return {0, 0, 0};
}

// ----------------------------------------------------------------- layer.py: sizes
inline double angular_to_metric(Shape shape, double alpha_deg, double distance, double radius)
{
    double a = alpha_deg * DEG / 2;
    if (shape == Shape::Flat) return 2 * distance * std::tan(a);
    double r = radius;
    double phi = a + std::asin(std::clamp(std::sin(a) * (distance - r) / r, -1.0, 1.0));
    return 2 * r * phi;
}

// ----------------------------------------------------------------- the resolved layer (Placement)
struct Placement {
    Vec3 center;
    Mat3 O;          // local -> world (columns: right, up, front normal)
    Shape shape = Shape::Flat;
    double width = 1, height = 1, radius = 0;
    double opacity = 1;
};

struct LayerParams {   // PlacedLayer at one time (keyframes already evaluated by MLT)
    double yaw = 0, pitch = 0, distance = 2;
    double width = 1, height = 0;      // height <= 0: from the texture aspect; width <= 0 (with height): idem
    bool angular = false;              // size_mode
    bool facing_viewer = true;
    double orient_yaw = 0, orient_pitch = 0, roll = 0;
    Shape shape = Shape::Flat;
    double curve_radius = 0;           // <= 0: the layer's distance
    double opacity = 1;
};

inline Placement resolve(const LayerParams& L, double texture_aspect)
{
    Placement P;
    P.center = L.distance * vrcore::lonlat_to_dir(L.yaw * DEG, L.pitch * DEG);
    const Vec3 c = P.center;
    double dist = std::sqrt(vrcore::dot(c, c));
    double lon = std::atan2(c.x, -c.z) / DEG, lat = std::atan2(c.y, std::hypot(c.x, c.z)) / DEG;
    P.O = L.facing_viewer ? vrcore::ypr_to_matrix(lon, lat, L.roll) : vrcore::ypr_to_matrix(L.orient_yaw, L.orient_pitch, L.roll);
    P.shape = L.shape;
    P.radius = L.shape == Shape::Flat ? 0 : (L.curve_radius > 0 ? L.curve_radius : dist);
    double w = L.width, h = L.height;
    bool hw = w > 0, hh = h > 0;
    if (L.angular) {
        if (hw) w = angular_to_metric(L.shape, w, dist, P.radius);
        if (hh) h = angular_to_metric(L.shape == Shape::Sphere ? Shape::Sphere : Shape::Flat, h, dist, P.radius);
    }
    if (!hh) h = w * texture_aspect;
    if (!hw) w = h / texture_aspect;
    P.width = w;
    P.height = h;
    P.opacity = L.opacity;
    return P;
}

// ----------------------------------------------------------------- surfaces.py: intersect in local coords
struct Hit {
    double t = INF, u = 0, v = 0;
    bool hit() const { return std::isfinite(t); }
};

inline void roots(Vec3 o, Vec3 d, double r, bool cylinder, double& t1, double& t2)
{
    double pz = o.z - r, a, b, c;
    if (cylinder) {
        a = d.x * d.x + d.z * d.z;
        b = 2 * (o.x * d.x + pz * d.z);
        c = o.x * o.x + pz * pz - r * r;
    } else {
        a = d.x * d.x + d.y * d.y + d.z * d.z;
        b = 2 * (o.x * d.x + o.y * d.y + pz * d.z);
        c = o.x * o.x + o.y * o.y + pz * pz - r * r;
    }
    double disc = b * b - 4 * a * c;
    bool ok = disc >= 0 && a > EPS;
    double sq = std::sqrt(ok ? disc : 0.0);
    double qq = -0.5 * (b + (b >= 0 ? sq : -sq));   // stable pair of roots
    t1 = ok ? qq / (std::fabs(a) > EPS ? a : 1.0) : INF;
    t2 = ok && std::fabs(qq) > EPS ? c / qq : INF;
}

inline Hit intersect(const Placement& P, Vec3 o, Vec3 d)   // local ray
{
    Hit h;
    if (P.shape == Shape::Flat) {
        double t = std::fabs(d.z) > EPS ? -o.z / d.z : INF;
        if (!std::isfinite(t) || t <= 0) return h;
        double u = (o.x + t * d.x) / P.width + 0.5, v = 0.5 - (o.y + t * d.y) / P.height;
        if (u >= 0 && u <= 1 && v >= 0 && v <= 1) h = {t, u, v};
        return h;
    }
    double t1, t2, r = P.radius;
    roots(o, d, r, P.shape == Shape::Cylinder, t1, t2);
    for (double t : {t1, t2}) {
        if (!std::isfinite(t)) continue;
        Vec3 p = o + t * d;
        double u, v;
        if (P.shape == Shape::Cylinder) {
            u = std::atan2(p.x, r - p.z) / (P.width / r) + 0.5;
            v = 0.5 - p.y / P.height;
        } else {
            double lon = std::atan2(p.x, r - p.z), lat = std::atan2(p.y, std::hypot(p.x, r - p.z));
            u = lon * r / P.width + 0.5;
            v = 0.5 - lat * r / P.height;
        }
        if (t > 0 && u >= 0 && u <= 1 && v >= 0 && v <= 1 && t < h.t) h = {t, u, v};
    }
    return h;
}

// world ray -> hit (Placement.intersect_world): local = O^T (x - c)
inline Hit intersect_world(const Placement& P, Vec3 o, Vec3 d)
{
    Mat3 Ot = P.O.transposed();
    return intersect(P, Ot * (o - P.center), Ot * d);
}

// ----------------------------------------------------------------- remap.sample on a premultiplied double texture
// (bilinear, pixel centres at +0.5, edge-clamped within half a pixel, 0 outside), 4 channels.
inline void sample_tex(const double* tex, int w, int h, double u, double v, double out[4])
{
    if (!(std::isfinite(u) && std::isfinite(v)) || u < 0 || u > w || v < 0 || v > h) {
        out[0] = out[1] = out[2] = out[3] = 0;
        return;
    }
    double x = u - 0.5, y = v - 0.5;
    int xi = (int) std::floor(x), yi = (int) std::floor(y);
    double fx = x - xi, fy = y - yi;
    auto cx = [&](int i) { return std::clamp(i, 0, w - 1); };
    auto cy = [&](int j) { return std::clamp(j, 0, h - 1); };
    const double* a = tex + 4 * (cy(yi) * w + cx(xi));
    const double* b = tex + 4 * (cy(yi) * w + cx(xi + 1));
    const double* c = tex + 4 * (cy(yi + 1) * w + cx(xi));
    const double* e = tex + 4 * (cy(yi + 1) * w + cx(xi + 1));
    for (int k = 0; k < 4; ++k) out[k] = (a[k] * (1 - fx) + b[k] * fx) * (1 - fy) + (c[k] * (1 - fx) + e[k] * fx) * fy;
}

}  // namespace vrov
