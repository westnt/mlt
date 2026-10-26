/*
 * filter_vr_eyealign.cpp -- correct a stereo lens misalignment: rotate the eyes relative to each other.
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.  Reference: vredit/view/eyealign.py (correct())
 * yaw/pitch/roll: the rotation Q estimated for the RIGHT eye (vredit.view.eyealign.estimate_misalignment).
 * split=1 (default): each eye gets half (L the inverse half, R the forward half), as the reference does.
 */
#include "vr_filter_common.hpp"
#include <cstring>

// Half of a rotation: quaternion of Q (w >= 0), halved by normalising (1 + w, x, y, z) == slerp(I, q, 0.5).
static vrcore::Mat3 half_rotation(const vrcore::Mat3 &Q)
{
    const double *R = Q.m;
    double w, x, y, z, tr = R[0] + R[4] + R[8];
    if (tr > 0) {
        double s = std::sqrt(tr + 1.0) * 2;
        w = 0.25 * s; x = (R[7] - R[5]) / s; y = (R[2] - R[6]) / s; z = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2;
        w = (R[7] - R[5]) / s; x = 0.25 * s; y = (R[1] + R[3]) / s; z = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2;
        w = (R[2] - R[6]) / s; x = (R[1] + R[3]) / s; y = 0.25 * s; z = (R[5] + R[7]) / s;
    } else {
        double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2;
        w = (R[3] - R[1]) / s; x = (R[2] + R[6]) / s; y = (R[5] + R[7]) / s; z = 0.25 * s;
    }
    if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
    w += 1.0;
    double n = std::sqrt(w * w + x * x + y * y + z * z);
    w /= n; x /= n; y /= n; z /= n;
    vrcore::Mat3 H;
    double v[9] = {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                   2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                   2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
    std::memcpy(H.m, v, sizeof v);
    return H;
}

static int vr_eyealign_get_image(mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width,
                                 int *height, int writable)
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
           roll = vr_anim(p, "roll", pos, len, 0);
    int split = mlt_properties_get(p, "split") ? mlt_properties_get_int(p, "split") : 1;
    mlt_service_unlock(MLT_FILTER_SERVICE(filter));
    if (L.eyes() < 2 || (yaw == 0 && pitch == 0 && roll == 0)) return 0;
    vrcore::Mat3 Q = vrcore::ypr_to_matrix(yaw, pitch, roll), I;
    vrcore::Mat3 QR = split ? half_rotation(Q) : Q;
    vrcore::Mat3 QL = split ? QR.transposed() : I;
    // the aligned eye samples the source at Q^T d
    vrcore::Mat3 RL = QL.transposed(), RR = QR.transposed();
    int size = *width * *height * 4;  // RGBA8 (mlt_image_format_size is deprecated)
    uint8_t *out = (uint8_t *) mlt_pool_alloc(size);
    const uint8_t *src = *image;
    vr_run_rows(L.eye_height, [&](int r0, int r1) {
        vrcore::transform_eye(src, out, L, 0, RL, r0, r1);
        vrcore::transform_eye(src, out, L, 1, RR, r0, r1);
    });
    mlt_frame_set_image(frame, out, size, mlt_pool_release);
    *image = out;
    return 0;
}

VR_FILTER_BOILERPLATE(vr_eyealign, vr_eyealign_get_image, mlt_properties_set_int(p, "split", 1))
