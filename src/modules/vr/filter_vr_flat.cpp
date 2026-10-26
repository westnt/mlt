/*
 * filter_vr_flat.cpp -- reframe a VR clip into flat video with a keyframed virtual camera (vredit/reframe/flat.py).
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.
 * Output keeps the frame size (set the project to 16:9 or 9:16). yaw/pitch/roll/hfov animated. eye: L or R.
 * stereo=1: half side-by-side flat 3D, each eye from its own eye with horizontal image translation so things at
 * `convergence` meters sit on the screen plane (baseline in meters, default 0.063). Point-sampled half width.
 */
#include "vr_filter_common.hpp"
#include <cstring>

static int vr_flat_get_image(mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width, int *height,
                             int writable)
{
    mlt_filter filter = (mlt_filter) mlt_frame_pop_service(frame);
    mlt_properties p = MLT_FILTER_PROPERTIES(filter);
    mlt_position pos = mlt_filter_get_position(filter, frame), len = mlt_filter_get_length2(filter, frame);
    *format = mlt_image_rgba;
    int err = mlt_frame_get_image(frame, image, format, width, height, 0);
    if (err || !*image) return err;
    mlt_service_lock(MLT_FILTER_SERVICE(filter));
    vrcore::Layout L = vr_layout_from(p, *width, *height);
    double yaw = vr_anim(p, "yaw", pos, len, 0), pitch = vr_anim(p, "pitch", pos, len, 0),
           roll = vr_anim(p, "roll", pos, len, 0), hfov = vr_anim(p, "hfov", pos, len, 90);
    const char *e = mlt_properties_get(p, "eye");
    int eye = e && !strcmp(e, "R") ? 1 : 0;
    int stereo = mlt_properties_get_int(p, "stereo") && L.eyes() == 2;
    double conv = vr_anim(p, "convergence", pos, len, 2.0);
    double base = mlt_properties_get(p, "baseline") ? mlt_properties_get_double(p, "baseline") : 0.063;
    mlt_service_unlock(MLT_FILTER_SERVICE(filter));
    const int W = *width, H = *height;
    vrcore::Mat3 R = vrcore::ypr_to_matrix(yaw, pitch, roll);
    vrcore::Rectilinear view{W, H, hfov};
    vrcore::Equirect pr = L.eye_projection();
    const int SW = L.packing == vrcore::Packing::SBS ? 2 * L.eye_width : L.eye_width;
    double shift = conv > 0 ? view.focal() * base / (2.0 * conv) : 0.0;
    int size = W * H * 4;  // RGBA8
    uint8_t *out = (uint8_t *) mlt_pool_alloc(size);
    const uint8_t *src = *image;
    const uint8_t black[4] = {0, 0, 0, 255};
    vr_run_rows(H, [&](int r0, int r1) {
        for (int j = r0; j < r1; ++j)
            for (int i = 0; i < W; ++i) {
                int ey = eye;
                double u = i + 0.5, sgn = 0;
                if (stereo) {  // left half = L eye, right half = R eye, each at full view width
                    ey = i < W / 2 ? 0 : 1;
                    u = 2.0 * ((ey ? i - W / 2 : i) + 0.5);
                    sgn = ey ? 1.0 : -1.0;
                }
                vrcore::Vec3 d;
                view.pixel_to_dir(u - sgn * shift, j + 0.5, d);
                double su, sv;
                uint8_t *o = out + 4 * (j * W + i);
                int x0, y0;
                L.eye_origin(ey, x0, y0);
                if (!pr.dir_to_pixel(R * d, su, sv)) { std::memcpy(o, black, 4); continue; }
                vrcore::sample_rgba8(src, SW, x0, y0, L.eye_width, L.eye_height, su, sv, pr.full360(), o, black);
            }
    });
    mlt_frame_set_image(frame, out, size, mlt_pool_release);
    *image = out;
    return 0;
}

VR_FILTER_BOILERPLATE(vr_flat, vr_flat_get_image, mlt_properties_set_double(p, "hfov", 90))
