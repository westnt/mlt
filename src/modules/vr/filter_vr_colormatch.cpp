/*
 * filter_vr_colormatch.cpp -- apply a per-eye colour correction estimated by vredit (view/colormatch.py).
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.
 * matrix: 12 numbers, a 3x4 row-major affine map on RGB in 0..1: rgb' = M[:, :3] rgb + M[:, 3]
 * (vredit.view.colormatch.affine_fit). eye: R (default) or L - which eye to correct. strength: 0..1 blend.
 */
#include "vr_filter_common.hpp"
#include <cstdlib>
#include <cstring>

static int vr_colormatch_get_image(mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width,
                                   int *height, int writable)
{
    mlt_filter filter = (mlt_filter) mlt_frame_pop_service(frame);
    mlt_properties p = MLT_FILTER_PROPERTIES(filter);
    mlt_position pos = mlt_filter_get_position(filter, frame), len = mlt_filter_get_length2(filter, frame);
    *format = mlt_image_rgba;
    int err = mlt_frame_get_image(frame, image, format, width, height, 1);
    if (err || !*image) return err;
    mlt_service_lock(MLT_FILTER_SERVICE(filter));
    vrcore::Layout L = vr_layout_from(p, *width, *height);
    double M[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    const char *ms = mlt_properties_get(p, "matrix");
    if (ms) {
        char *end = (char *) ms;
        for (int k = 0; k < 12; ++k) M[k] = strtod(end, &end);
    }
    const char *e = mlt_properties_get(p, "eye");
    int eye = e && !strcmp(e, "L") ? 0 : 1;
    double strength = vr_anim(p, "strength", pos, len, 1.0);
    mlt_service_unlock(MLT_FILTER_SERVICE(filter));
    if (L.eyes() < 2) return 0;
    const int W = *width;
    uint8_t *img = *image;
    int x0, y0;
    L.eye_origin(eye, x0, y0);
    vr_run_rows(L.eye_height, [&](int r0, int r1) {
        for (int j = r0; j < r1; ++j)
            for (int i = 0; i < L.eye_width; ++i) {
                uint8_t *px = img + 4 * ((y0 + j) * W + x0 + i);
                double c[3] = {px[0] / 255.0, px[1] / 255.0, px[2] / 255.0};
                for (int ch = 0; ch < 3; ++ch) {
                    double v = M[4 * ch] * c[0] + M[4 * ch + 1] * c[1] + M[4 * ch + 2] * c[2] + M[4 * ch + 3];
                    v = strength * v + (1 - strength) * c[ch];
                    px[ch] = (uint8_t) std::clamp((int) std::lround(v * 255.0), 0, 255);
                }
            }
    });
    return 0;
}

VR_FILTER_BOILERPLATE(vr_colormatch, vr_colormatch_get_image, mlt_properties_set(p, "eye", "R"))
