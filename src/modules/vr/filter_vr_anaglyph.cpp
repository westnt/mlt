/*
 * filter_vr_anaglyph.cpp -- red/cyan Dubois anaglyph of a stereo VR clip, for checking stereo in Shotcut's player
 * without a headset (vredit/preview/views.py; matrices from ffmpeg's stereo3d, bit-exact). Output = one eye's size.
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.
 */
#include "vr_filter_common.hpp"

static int vr_anaglyph_get_image(mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width,
                                 int *height, int writable)
{
    mlt_filter filter = (mlt_filter) mlt_frame_pop_service(frame);
    mlt_properties p = MLT_FILTER_PROPERTIES(filter);
    *format = mlt_image_rgba;
    int err = mlt_frame_get_image(frame, image, format, width, height, 0);
    if (err || !*image) return err;
    mlt_service_lock(MLT_FILTER_SERVICE(filter));
    vrcore::Layout L = vr_layout_from(p, *width, *height);
    mlt_service_unlock(MLT_FILTER_SERVICE(filter));
    if (L.eyes() < 2) return 0;
    const int W = *width, ew = L.eye_width, eh = L.eye_height;
    int lx, ly, rx, ry;
    L.eye_origin(0, lx, ly);
    L.eye_origin(1, rx, ry);
    int size = ew * eh * 4;  // RGBA8
    uint8_t *out = (uint8_t *) mlt_pool_alloc(size);
    const uint8_t *src = *image;
    vr_run_rows(eh, [&](int r0, int r1) {
        for (int j = r0; j < r1; ++j)
            for (int i = 0; i < ew; ++i) {
                uint8_t *o = out + 4 * (j * ew + i);
                vrcore::anaglyph_dubois_rc(src + 4 * ((ly + j) * W + lx + i), src + 4 * ((ry + j) * W + rx + i), o);
                o[3] = 255;
            }
    });
    mlt_frame_set_image(frame, out, size, mlt_pool_release);
    *image = out;
    *width = ew;
    *height = eh;
    mlt_properties_set_int(MLT_FRAME_PROPERTIES(frame), "width", ew);
    mlt_properties_set_int(MLT_FRAME_PROPERTIES(frame), "height", eh);
    return 0;
}

VR_FILTER_BOILERPLATE(vr_anaglyph, vr_anaglyph_get_image, (void) 0)
