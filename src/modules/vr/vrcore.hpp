// vrcore.hpp - header-only C++17 port of vredit.core (Python reference in vredit/core/*.py).
//
// Line-by-line port: same function names, same conventions, same math, so every function can be pixel-diffed
// against the reference (port/tests). Read README.md "Conventions" first:
//   * OpenXR axes: +x right, +y up, viewer looks along -z. lon = atan2(x, -z), lat = asin(y).
//   * yaw/pitch/roll = where the viewer looks; R = Ry(-yaw) Rx(pitch) Rz(-roll); rendering maps output view
//     directions to source directions with R (inverse map per output pixel).
//   * Pixel (i, j) has its center at (i + 0.5, j + 0.5); v grows downward.
// License: GPL-2.0-or-later (to match Shotcut/MLT plugin distribution); math derived from vredit (same project).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace vrcore {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG = PI / 180.0;

struct Vec3 {
    double x = 0, y = 0, z = 0;
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(double s, Vec3 a) { return {s * a.x, s * a.y, s * a.z}; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 normalize(Vec3 v) {
    double n = std::sqrt(dot(v, v));
    return n > 0 ? (1.0 / n) * v : v;
}

// Row-major 3x3, acting on column vectors: d' = M d
struct Mat3 {
    double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    Vec3 operator*(Vec3 v) const {
        return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
                m[6] * v.x + m[7] * v.y + m[8] * v.z};
    }
    Mat3 operator*(const Mat3& b) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double s = 0;
                for (int k = 0; k < 3; ++k) s += m[3 * i + k] * b.m[3 * k + j];
                r.m[3 * i + j] = s;
            }
        return r;
    }
    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[3 * i + j] = m[3 * j + i];
        return r;
    }
};

// ------------------------------------------------------------------ sphere.py
inline Vec3 lonlat_to_dir(double lon, double lat) {
    double cl = std::cos(lat);
    return {cl * std::sin(lon), std::sin(lat), -cl * std::cos(lon)};
}
inline void dir_to_lonlat(Vec3 d, double& lon, double& lat) {
    lon = std::atan2(d.x, -d.z);
    lat = std::atan2(d.y, std::hypot(d.x, d.z));
}
inline Mat3 rot_x(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    double v[9] = {1, 0, 0, 0, c, -s, 0, s, c};
    std::memcpy(r.m, v, sizeof v);
    return r;
}
inline Mat3 rot_y(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    double v[9] = {c, 0, s, 0, 1, 0, -s, 0, c};
    std::memcpy(r.m, v, sizeof v);
    return r;
}
inline Mat3 rot_z(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    double v[9] = {c, -s, 0, s, c, 0, 0, 0, 1};
    std::memcpy(r.m, v, sizeof v);
    return r;
}
inline Mat3 ypr_to_matrix(double yaw_deg, double pitch_deg, double roll_deg) {
    return rot_y(-yaw_deg * DEG) * rot_x(pitch_deg * DEG) * rot_z(-roll_deg * DEG);
}

// ------------------------------------------------------------- projections.py
struct Equirect {
    int width = 0, height = 0;
    double lon0 = -180, lon1 = 180, lat0 = -90, lat1 = 90;  // degrees
    bool full360() const { return std::fabs((lon1 - lon0) - 360.0) < 1e-9; }
    bool pixel_to_dir(double u, double v, Vec3& d) const {
        double lon = lon0 * DEG + (u / width) * (lon1 - lon0) * DEG;
        double lat = lat1 * DEG - (v / height) * (lat1 - lat0) * DEG;
        d = lonlat_to_dir(lon, lat);
        return u >= 0 && u <= width && v >= 0 && v <= height;
    }
    bool dir_to_pixel(Vec3 d, double& u, double& v) const {
        double lon, lat;
        dir_to_lonlat(d, lon, lat);
        double lo0 = lon0 * DEG, lo1 = lon1 * DEG, la0 = lat0 * DEG, la1 = lat1 * DEG;
        if (full360()) lon = lo0 + std::fmod(std::fmod(lon - lo0, 2 * PI) + 2 * PI, 2 * PI);
        u = (lon - lo0) / (lo1 - lo0) * width;
        v = (la1 - lat) / (la1 - la0) * height;
        const double eps = 1e-9;
        return lon >= lo0 - eps && lon <= lo1 + eps && lat >= la0 - eps && lat <= la1 + eps;
    }
};

struct Fisheye {  // equidistant model with optional k1..k4 (Kannala-Brandt / cv2.fisheye), as the reference
    int width = 0, height = 0;
    double fov = 190;            // degrees
    double cx = -1, cy = -1;     // <0: image center
    double radius = -1;          // <0: min(w, h) / 2
    double fx = -1, fy = -1;     // <0: from radius
    double k[4] = {0, 0, 0, 0};
    double distort(double t) const {
        double t2 = t * t;
        return t * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
    }
    double undistort(double td) const {
        if (!k[0] && !k[1] && !k[2] && !k[3]) return td;
        double t = td;
        for (int i = 0; i < 20; ++i) {
            double t2 = t * t;
            double f = t * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3])))) - td;
            double df = 1 + t2 * (3 * k[0] + t2 * (5 * k[1] + t2 * (7 * k[2] + t2 * 9 * k[3])));
            t -= f / df;
        }
        return t;
    }
    void focal(double& fxo, double& fyo, double& cxo, double& cyo) const {
        cxo = cx >= 0 ? cx : width / 2.0;
        cyo = cy >= 0 ? cy : height / 2.0;
        if (fx > 0) {
            fxo = fx;
            fyo = fy > 0 ? fy : fx;
            return;
        }
        double r = radius > 0 ? radius : std::min(width, height) / 2.0;
        fxo = fyo = r / distort(fov / 2 * DEG);
    }
    bool pixel_to_dir(double u, double v, Vec3& d) const {
        double fxx, fyy, cxx, cyy;
        focal(fxx, fyy, cxx, cyy);
        double dx = (u - cxx) / fxx, dy = (v - cyy) / fyy;
        double theta = undistort(std::hypot(dx, dy));
        double phi = std::atan2(-dy, dx);
        double st = std::sin(theta);
        d = {st * std::cos(phi), st * std::sin(phi), -std::cos(theta)};
        return theta <= fov / 2 * DEG + 1e-9;
    }
    bool dir_to_pixel(Vec3 d, double& u, double& v) const {
        double fxx, fyy, cxx, cyy;
        focal(fxx, fyy, cxx, cyy);
        d = normalize(d);
        double theta = std::atan2(std::hypot(d.x, d.y), -d.z);
        double phi = std::atan2(d.y, d.x);
        double r = distort(theta);
        u = cxx + fxx * r * std::cos(phi);
        v = cyy - fyy * r * std::sin(phi);
        return theta <= fov / 2 * DEG + 1e-9;
    }
};

struct Rectilinear {
    int width = 0, height = 0;
    double hfov = 90;
    double focal() const { return (width / 2.0) / std::tan(hfov * DEG / 2); }
    bool pixel_to_dir(double u, double v, Vec3& d) const {
        double f = focal();
        d = normalize({(u - width / 2.0) / f, (height / 2.0 - v) / f, -1.0});
        return true;
    }
    bool dir_to_pixel(Vec3 d, double& u, double& v) const {
        double z = -d.z;
        if (z <= 1e-12) return false;
        double f = focal();
        u = width / 2.0 + f * d.x / z;
        v = height / 2.0 - f * d.y / z;
        return true;
    }
};

// ------------------------------------------------------------------- layout.py
enum class Packing { Mono, SBS, TB };
struct Layout {
    Packing packing = Packing::SBS;
    int eye_width = 0, eye_height = 0;
    double lon0 = -90, lon1 = 90, lat0 = -90, lat1 = 90;
    bool swap_eyes = false;
    int eyes() const { return packing == Packing::Mono ? 1 : 2; }
    Equirect eye_projection() const { return {eye_width, eye_height, lon0, lon1, lat0, lat1}; }
    // eye 0 = L, 1 = R; returns x0, y0 of that eye in the packed frame
    void eye_origin(int eye, int& x0, int& y0) const {
        bool first = (eye == 0) != swap_eyes;
        x0 = y0 = 0;
        if (packing == Packing::SBS && !first) x0 = eye_width;
        if (packing == Packing::TB && !first) y0 = eye_height;
    }
};

// -------------------------------------------------------------------- remap.py
// Bilinear sample of an RGBA8 image region (pixel centers at +0.5), edge-clamped within half a pixel, border
// outside. Matches vredit.core.remap.sample on 8-bit data (up to rounding of the final value).
inline void sample_rgba8(const uint8_t* img, int stride_px, int x0, int y0, int w, int h, double u, double v,
                         bool wrap_u, uint8_t out[4], const uint8_t border[4]) {
    bool outside = !(std::isfinite(u) && std::isfinite(v)) || v < 0 || v > h || (!wrap_u && (u < 0 || u > w));
    if (outside) {
        std::memcpy(out, border, 4);
        return;
    }
    double x = u - 0.5, y = v - 0.5;
    int xi = (int) std::floor(x), yi = (int) std::floor(y);
    double fx = x - xi, fy = y - yi;
    auto cx = [&](int i) { return wrap_u ? ((i % w) + w) % w : std::clamp(i, 0, w - 1); };
    auto cy = [&](int j) { return std::clamp(j, 0, h - 1); };
    const uint8_t* a = img + 4 * ((y0 + cy(yi)) * stride_px + x0 + cx(xi));
    const uint8_t* b = img + 4 * ((y0 + cy(yi)) * stride_px + x0 + cx(xi + 1));
    const uint8_t* c = img + 4 * ((y0 + cy(yi + 1)) * stride_px + x0 + cx(xi));
    const uint8_t* d = img + 4 * ((y0 + cy(yi + 1)) * stride_px + x0 + cx(xi + 1));
    for (int ch = 0; ch < 4; ++ch) {
        double val = (a[ch] * (1 - fx) + b[ch] * fx) * (1 - fy) + (c[ch] * (1 - fx) + d[ch] * fx) * fy;
        out[ch] = (uint8_t) std::clamp((int) std::lround(val), 0, 255);
    }
}

// --------------------------------------------------------------------- warp.py
// Rotate every eye of a packed RGBA8 frame by R (view -> source). src and dst may not alias.
inline void transform_frame(const uint8_t* src, uint8_t* dst, const Layout& L, const Mat3& R, int row_begin = 0,
                            int row_end = -1, bool transparent = true) {
    Equirect p = L.eye_projection();
    const int W = L.packing == Packing::SBS ? 2 * L.eye_width : L.eye_width;
    // Outside the picture: transparent (default) so tracks below show through, or opaque black.
    const uint8_t black[4] = {0, 0, 0, uint8_t(transparent ? 0 : 255)};
    for (int e = 0; e < L.eyes(); ++e) {
        int x0, y0;
        L.eye_origin(e, x0, y0);
        int r1 = row_end < 0 ? L.eye_height : std::min(row_end, L.eye_height);
        for (int j = row_begin; j < r1; ++j)
            for (int i = 0; i < L.eye_width; ++i) {
                Vec3 d;
                bool ok = p.pixel_to_dir(i + 0.5, j + 0.5, d);
                double su, sv;
                ok = ok && p.dir_to_pixel(R * d, su, sv);
                uint8_t* o = dst + 4 * ((y0 + j) * W + x0 + i);
                if (!ok) {
                    std::memcpy(o, black, 4);
                    continue;
                }
                sample_rgba8(src, W, x0, y0, L.eye_width, L.eye_height, su, sv, p.full360(), o, black);
            }
    }
}

// ------------------------------------------------- per-eye rotation (eyealign.py, any per-eye op)
// Rotate one eye (0 = L, 1 = R) of a packed frame by R; the other eye is copied unchanged by the caller.
inline void transform_eye(const uint8_t* src, uint8_t* dst, const Layout& L, int eye, const Mat3& R, int row_begin = 0,
                          int row_end = -1, bool transparent = true) {
    Equirect p = L.eye_projection();
    const int W = L.packing == Packing::SBS ? 2 * L.eye_width : L.eye_width;
    // Outside the picture: transparent (default) so tracks below show through, or opaque black.
    const uint8_t black[4] = {0, 0, 0, uint8_t(transparent ? 0 : 255)};
    int x0, y0;
    L.eye_origin(eye, x0, y0);
    int r1 = row_end < 0 ? L.eye_height : std::min(row_end, L.eye_height);
    for (int j = row_begin; j < r1; ++j)
        for (int i = 0; i < L.eye_width; ++i) {
            Vec3 d;
            bool ok = p.pixel_to_dir(i + 0.5, j + 0.5, d);
            double su, sv;
            ok = ok && p.dir_to_pixel(R * d, su, sv);
            uint8_t* o = dst + 4 * ((y0 + j) * W + x0 + i);
            if (!ok) {
                std::memcpy(o, black, 4);
                continue;
            }
            sample_rgba8(src, W, x0, y0, L.eye_width, L.eye_height, su, sv, p.full360(), o, black);
        }
}

// --------------------------------------------------------------------- masks.py
inline double smoothstep(double e0, double e1, double x) {
    double t = std::clamp((x - e0) / (e1 != e0 ? e1 - e0 : 1e-12), 0.0, 1.0);
    return t * t * (3 - 2 * t);
}
inline double angle_between(Vec3 a, Vec3 b) {
    a = normalize(a);
    b = normalize(b);
    Vec3 c{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    return std::atan2(std::sqrt(dot(c, c)), dot(a, b));
}
// Weight (1 = keep) of the FOV circle mask at an eye pixel center (masks.fov_mask).
inline double fov_mask_at(const Equirect& p, double u, double v, double fov_deg, double feather_deg, double clon,
                          double clat) {
    Vec3 d;
    if (!p.pixel_to_dir(u, v, d)) return 0.0;
    double a = angle_between(d, lonlat_to_dir(clon * DEG, clat * DEG)) / DEG;
    return 1.0 - smoothstep(fov_deg / 2 - feather_deg, fov_deg / 2, a);
}
// Longitude/latitude window (masks.rect_mask).
inline double rect_mask_at(const Equirect& p, double u, double v, double lo0, double lo1, double la0, double la1,
                           double f) {
    Vec3 d;
    if (!p.pixel_to_dir(u, v, d)) return 0.0;
    double lo, la;
    dir_to_lonlat(d, lo, la);
    lo /= DEG;
    la /= DEG;
    return smoothstep(lo0, lo0 + f, lo) * (1 - smoothstep(lo1 - f, lo1, lo)) * smoothstep(la0, la0 + f, la) *
           (1 - smoothstep(la1 - f, la1, la));
}
// Mono nadir/zenith cap weight (masks.nadir_cap with floor_height=None): disc around the pole.
inline double cap_at(const Equirect& p, double u, double v, double radius_deg, double feather_deg, bool zenith) {
    Vec3 d;
    if (!p.pixel_to_dir(u, v, d)) return 0.0;
    double sgn = zenith ? 1.0 : -1.0;
    if (d.y * sgn <= 0) return 0.0;
    double t = sgn / d.y;  // ray to the plane y = sgn (h = 1)
    double rr = std::hypot(t * d.x, t * d.z);
    double R = std::tan(radius_deg * DEG), Rf = std::tan((radius_deg + feather_deg) * DEG);
    return 1.0 - smoothstep(R, Rf, rr);
}

// ------------------------------------------------------------- preview/views.py
// ffmpeg stereo3d ana_coeff, rows out R,G,B; cols L.rgb, R.rgb; / 65536 with floor shift (bit-exact with ffmpeg).
inline void anaglyph_dubois_rc(const uint8_t* l, const uint8_t* r, uint8_t* out) {
    static const int M[3][6] = {{29884, 32768, 11534, -2818, -5767, -131},
                                {-2621, -2490, -1049, 24773, 48103, -1180},
                                {-983, -1376, -328, -4719, -7406, 80347}};
    for (int c = 0; c < 3; ++c) {
        int s = M[c][0] * l[0] + M[c][1] * l[1] + M[c][2] * l[2] + M[c][3] * r[0] + M[c][4] * r[1] + M[c][5] * r[2];
        out[c] = (uint8_t) std::clamp(s >> 16, 0, 255);
    }
}

}  // namespace vrcore
