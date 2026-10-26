/*
 * filter_vr_mask.cpp -- per-eye masks for VR clips: FOV circle, lon/lat window, nadir / zenith cap.
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.  Reference: vredit/view/masks.py
 * mode: fov (keep a circle of `fov` degrees around center_lon/center_lat), rect (keep the window
 * crop_lon_min..crop_lon_max x crop_lat_min..crop_lat_max), nadir / zenith (cover a cap of cap_radius degrees).
 * Outside the kept area (or inside the cap) the picture blends to `color` over `feather` degrees.
 */
#include "vr_filter_common.hpp"
#include <cstring>

static int vr_mask_get_image(mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width, int *height,
                             int writable)
{
    mlt_filter filter = (mlt_filter) mlt_frame_pop_service(frame);
    mlt_properties p = MLT_FILTER_PROPERTIES(filter);
    mlt_position pos = mlt_filter_get_position(filter, frame), len = mlt_filter_get_length2(filter, frame);
    *format = mlt_image_rgba;
    int err = mlt_frame_get_image(frame, image, format, width, height, 1);
    if (err || !*image) return err;
    mlt_service_lock(MLT_FILTER_SERVICE(filter));
    vrcore::Layout L = vr_layout_from(p, *width, *height);
    const char *m = mlt_properties_get(p, "mode");
    int mode = !m || !strcmp(m, "fov") ? 0 : !strcmp(m, "rect") ? 1 : !strcmp(m, "zenith") ? 3 : 2;
    double fov = vr_anim(p, "fov", pos, len, 180), feather = vr_anim(p, "feather", pos, len, 5);
    double clon = vr_anim(p, "center_lon", pos, len, 0), clat = vr_anim(p, "center_lat", pos, len, 0);
    double lo0 = vr_anim(p, "crop_lon_min", pos, len, -90), lo1 = vr_anim(p, "crop_lon_max", pos, len, 90);
    double la0 = vr_anim(p, "crop_lat_min", pos, len, -90), la1 = vr_anim(p, "crop_lat_max", pos, len, 90);
    double cap = vr_anim(p, "cap_radius", pos, len, 20);
    mlt_color c = mlt_properties_get(p, "color") ? mlt_properties_get_color(p, "color") : mlt_color{0, 0, 0, 255};
    mlt_service_unlock(MLT_FILTER_SERVICE(filter));
    const int W = *width;
    uint8_t *img = *image;
    vrcore::Equirect pr = L.eye_projection();
    const double col[3] = {(double) c.r, (double) c.g, (double) c.b};
    vr_run_rows(L.eye_height, [&](int r0, int r1) {
        for (int e = 0; e < L.eyes(); ++e) {
            int x0, y0;
            L.eye_origin(e, x0, y0);
            for (int j = r0; j < r1; ++j)
                for (int i = 0; i < L.eye_width; ++i) {
                    double u = i + 0.5, v = j + 0.5, keep;
                    if (mode == 0) keep = vrcore::fov_mask_at(pr, u, v, fov, feather, clon, clat);
                    else if (mode == 1) keep = vrcore::rect_mask_at(pr, u, v, lo0, lo1, la0, la1, feather);
                    else keep = 1.0 - vrcore::cap_at(pr, u, v, cap, feather, mode == 3);
                    uint8_t *px = img + 4 * ((y0 + j) * W + x0 + i);
                    for (int ch = 0; ch < 3; ++ch)
                        px[ch] = (uint8_t) std::clamp((int) std::lround(px[ch] * keep + col[ch] * (1 - keep)), 0, 255);
                }
        }
    });
    return 0;
}

VR_FILTER_BOILERPLATE(vr_mask, vr_mask_get_image, mlt_properties_set(p, "mode", "fov"))
