/*
 * filter_vr_convergence.cpp -- move the whole stereo scene nearer / farther: opposite shift per eye.
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.
 * Reference: vredit/stereo3d/convergence.py (source_dirs, Convergence.apply); checked pixel by pixel in
 * port/tests/test_port_convergence.py.
 *
 * shift (degrees, animated): total disparity change, + = scene nearer (the left eye's content moves right).
 * method: auto | yaw | epipolar.
 *   yaw       rigid rotation of each eye about the vertical axis by -+shift/2: exact for ODS (stereo 360).
 *   epipolar  parallel rigs (VR180): content moves along its epipolar circle (about the baseline axis) by
 *             -+(shift/2) sin(beta), so vertical disparity stays zero and the 180 border maps onto itself.
 *   auto      epipolar for parallel rigs, yaw for ODS.
 * rig: auto | parallel | ods (auto: ods when the eye covers 360 deg of longitude).
 * Convergence acts in the camera frame: put it BEFORE vr_transform in the filter chain (the reference applies the
 * view rotation first in its inverse map, which is the same thing).
 */
#include "vr_filter_common.hpp"
#include <cstring>

namespace {

// eye 0 = L, 1 = R. Content offset at the centre (radians): L moves right (+lon) for shift > 0.
inline double eye_offset(int eye, double shift_deg) { return (eye == 0 ? 1.0 : -1.0) * shift_deg * vrcore::DEG / 2; }

// source_dirs(): output direction (camera frame) -> source direction to sample.
inline vrcore::Vec3 source_dir(vrcore::Vec3 d, int eye, double shift_deg, bool epipolar)
{
    double o = eye_offset(eye, shift_deg);
    if (!epipolar) return vrcore::ypr_to_matrix(-o / vrcore::DEG, 0, 0) * d;
    d = vrcore::normalize(d);
    double alpha = std::atan2(d.y, -d.z);
    double b_out = std::atan2(std::hypot(d.y, d.z), d.x);
    double beta = b_out;  // solve beta - o sin(beta) = b_out (Kepler), Newton, 3 steps as the reference
    for (int k = 0; k < 3; ++k) beta -= (beta - o * std::sin(beta) - b_out) / (1 - o * std::cos(beta));
    double sb = std::sin(beta);
    return {std::cos(beta), sb * std::sin(alpha), -sb * std::cos(alpha)};
}

}  // namespace

static int vr_convergence_get_image(mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width,
                                    int *height, int writable)
{
    (void) writable;
    mlt_filter filter = (mlt_filter) mlt_frame_pop_service(frame);
    mlt_properties p = MLT_FILTER_PROPERTIES(filter);
    mlt_position pos = mlt_filter_get_position(filter, frame), len = mlt_filter_get_length2(filter, frame);
    *format = mlt_image_rgba;
    int err = mlt_frame_get_image(frame, image, format, width, height, 0);
    if (err || !*image) return err;
    mlt_service_lock(MLT_FILTER_SERVICE(filter));
    vrcore::Layout L = vr_layout_from(p, *width, *height);
    double shift = vr_anim(p, "shift", pos, len, 0);
    const char *m = mlt_properties_get(p, "method");
    const char *rg = mlt_properties_get(p, "rig");
    mlt_service_unlock(MLT_FILTER_SERVICE(filter));
    vrcore::Equirect proj = L.eye_projection();
    bool ods = rg && !strcmp(rg, "ods") ? true : rg && !strcmp(rg, "parallel") ? false : proj.full360();
    bool epipolar = m && !strcmp(m, "yaw") ? false : m && !strcmp(m, "epipolar") ? true : !ods;
    if (L.eyes() < 2 || shift == 0) return 0;
    const int W = L.packing == vrcore::Packing::SBS ? 2 * L.eye_width : L.eye_width;
    int size = *width * *height * 4;  // RGBA8
    uint8_t *out = (uint8_t *) mlt_pool_alloc(size);
    const uint8_t *src = *image;
    const uint8_t black[4] = {0, 0, 0, 255};
    vr_run_rows(L.eye_height, [&](int r0, int r1) {
        for (int e = 0; e < 2; ++e) {
            int x0, y0;
            L.eye_origin(e, x0, y0);
            for (int j = r0; j < r1; ++j)
                for (int i = 0; i < L.eye_width; ++i) {
                    vrcore::Vec3 d;
                    bool ok = proj.pixel_to_dir(i + 0.5, j + 0.5, d);
                    double su, sv;
                    ok = ok && proj.dir_to_pixel(source_dir(d, e, shift, epipolar), su, sv);
                    uint8_t *o = out + 4 * ((y0 + j) * W + x0 + i);
                    if (!ok) {
                        std::memcpy(o, black, 4);
                        continue;
                    }
                    vrcore::sample_rgba8(src, W, x0, y0, L.eye_width, L.eye_height, su, sv, proj.full360(), o, black);
                }
        }
    });
    mlt_frame_set_image(frame, out, size, mlt_pool_release);
    *image = out;
    return 0;
}

VR_FILTER_BOILERPLATE(vr_convergence, vr_convergence_get_image,
                      mlt_properties_set_double(p, "shift", 0); mlt_properties_set(p, "method", "auto");
                      mlt_properties_set(p, "rig", "auto"))
