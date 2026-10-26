/*
 * filter_vr_overlay.cpp -- the "vr_overlay" TRANSITION: place track B's picture in the 3D VR scene of track A.
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.
 * Reference: vredit/place/layer.py (PlacedLayer, render_layer_eye, composite_frame) and surfaces.py; checked pixel by
 * pixel in port/tests/test_port_overlay.py. (A transition, because in Shotcut/MLT the overlay is another track's
 * frame: title, image, video or a keyed subject. File named filter_* to sit with the other vr sources.)
 *
 * Also registered as a FILTER with the same name (vr_overlay_filter_get_image): the form Shotcut uses, see below.
 * A = the VR clip (any layout, 180 / 360, mono / stereo). B = the overlay picture (RGBA, straight alpha, any size; its
 * aspect sets the layer's height when height is 0). Per output pixel of each eye: that eye's ray (each eye from its
 * own centre, ODS rays from the ODS circle) hits the layer surface -> texture coordinates -> premultiplied bilinear
 * sample, ss x ss rays per pixel, times opacity, composited over A. Each eye's disparity comes from the geometry.
 *
 * Properties (angles in degrees, lengths in metres; the first nine are animated):
 *   yaw, pitch, distance            where the layer's centre is (around the viewer; keyframes = spherical path)
 *   width, height                   size (height 0 = from B's aspect); size_mode metric | angular
 *   roll, orient_yaw, orient_pitch  facing viewer | fixed (orient_* used when fixed)
 *   opacity                         0..1
 *   shape flat | cylinder | sphere, curve_radius (0 = the distance)
 *   rig auto | none | parallel | ods (auto: mono -> none, 360 -> ods, else parallel), baseline (m), ss (1..4)
 *   + the VR layout properties of every vr filter (packing, lon/lat range, swap_eyes)
 */
#include "filter_vr_overlay_math.hpp"
#include "vr_filter_common.hpp"
#include <cstring>
#include <vector>

static double anim(mlt_properties p, const char *k, mlt_position pos, mlt_position len, double d)
{
    return mlt_properties_get(p, k) ? mlt_properties_anim_get_double(p, k, pos, len) : d;
}

static bool is(mlt_properties p, const char *k, const char *v)
{
    const char *s = mlt_properties_get(p, k);
    return s && !strcmp(s, v);
}

struct OverlaySetup {
    vrcore::Layout L;
    vrov::LayerParams lp;
    double baseline = 0.063;
    int ss = 1;
    vrov::Rig rig = vrov::Rig::Parallel;
};

// Read the layer + layout properties at this frame (keyframes evaluated by MLT). width/height = the output frame.
static OverlaySetup read_setup(mlt_properties p, mlt_position pos, mlt_position len, int width, int height)
{
    OverlaySetup S;
    S.L = vr_layout_from(p, width, height);
    vrov::LayerParams &lp = S.lp;
    lp.yaw = anim(p, "yaw", pos, len, 0);
    lp.pitch = anim(p, "pitch", pos, len, 0);
    lp.distance = anim(p, "distance", pos, len, 2);
    lp.width = anim(p, "width", pos, len, 1);
    lp.height = anim(p, "height", pos, len, 0);
    lp.roll = anim(p, "roll", pos, len, 0);
    lp.orient_yaw = anim(p, "orient_yaw", pos, len, 0);
    lp.orient_pitch = anim(p, "orient_pitch", pos, len, 0);
    lp.opacity = anim(p, "opacity", pos, len, 1);
    lp.angular = is(p, "size_mode", "angular");
    lp.facing_viewer = !is(p, "facing", "fixed");
    lp.shape = is(p, "shape", "cylinder") ? vrov::Shape::Cylinder : is(p, "shape", "sphere") ? vrov::Shape::Sphere : vrov::Shape::Flat;
    lp.curve_radius = mlt_properties_get(p, "curve_radius") ? mlt_properties_get_double(p, "curve_radius") : 0;
    S.baseline = mlt_properties_get(p, "baseline") ? mlt_properties_get_double(p, "baseline") : 0.063;
    S.ss = std::clamp(mlt_properties_get(p, "ss") ? mlt_properties_get_int(p, "ss") : 1, 1, 4);
    const vrcore::Layout &L = S.L;
    S.rig = is(p, "rig", "none") ? vrov::Rig::None : is(p, "rig", "parallel") ? vrov::Rig::Parallel
            : is(p, "rig", "ods") ? vrov::Rig::ODS
            : L.eyes() < 2 ? vrov::Rig::None : L.eye_projection().full360() ? vrov::Rig::ODS : vrov::Rig::Parallel;
    return S;
}

// Render the layer (texture = straight 8-bit RGBA bw x bh) into img (the packed VR frame, RGBA8).
// over = true: composite premultiplied over img (the transition). over = false: img is overwritten with the layer
// alone, straight alpha, transparent where the layer isn't (the filter; Shotcut's track blending composites it).
static void render_overlay(const OverlaySetup &S, const uint8_t *bimg, int bw, int bh, uint8_t *img, bool over)
{
    std::vector<double> tex((size_t) bw * bh * 4);  // premultiplied, 0..1 (the reference composites premultiplied)
    for (size_t i = 0; i < (size_t) bw * bh; ++i) {
        double al = bimg[4 * i + 3] / 255.0;
        for (int c = 0; c < 3; ++c) tex[4 * i + c] = bimg[4 * i + c] / 255.0 * al;
        tex[4 * i + 3] = al;
    }
    const vrcore::Layout &L = S.L;
    const int ss = S.ss;
    vrov::Placement P = vrov::resolve(S.lp, (double) bh / bw);
    vrcore::Equirect proj = L.eye_projection();
    const int W = L.packing == vrcore::Packing::SBS ? 2 * L.eye_width : L.eye_width;
    vr_run_rows(L.eye_height, [&](int r0, int r1) {
        for (int e = 0; e < L.eyes(); ++e) {
            int x0, y0;
            L.eye_origin(e, x0, y0);
            for (int j = r0; j < r1; ++j)
                for (int i = 0; i < L.eye_width; ++i) {
                    double acc[4] = {0, 0, 0, 0};
                    for (int sy = 0; sy < ss; ++sy)
                        for (int sx = 0; sx < ss; ++sx) {
                            double du = (sx + 0.5) / ss - 0.5, dv = (sy + 0.5) / ss - 0.5;
                            vrcore::Vec3 d;
                            if (!proj.pixel_to_dir(i + 0.5 + du, j + 0.5 + dv, d)) continue;
                            vrov::Hit h = vrov::intersect_world(P, vrov::ray_origin(S.rig, S.baseline, e, d), d);
                            if (!h.hit()) continue;
                            double s[4];
                            vrov::sample_tex(tex.data(), bw, bh, h.u * bw, h.v * bh, s);
                            for (int k = 0; k < 4; ++k) acc[k] += s[k];
                        }
                    double a = acc[3] / (ss * ss) * P.opacity;
                    uint8_t *o = img + 4 * ((y0 + j) * W + x0 + i);
                    if (!over) {  // the layer alone, straight alpha; no colour hidden in fully transparent pixels
                        o[3] = (uint8_t) std::clamp((int) std::lround(255.0 * a), 0, 255);
                        for (int k = 0; k < 3; ++k)
                            o[k] = o[3] ? (uint8_t) std::clamp((int) std::lround(255.0 * acc[k] / (ss * ss) * P.opacity / a), 0, 255) : 0;
                        continue;
                    }
                    if (a <= 0) continue;
                    for (int k = 0; k < 3; ++k) {
                        double top = acc[k] / (ss * ss) * P.opacity;  // premultiplied over A
                        o[k] = (uint8_t) std::clamp((int) std::lround(255.0 * (top + o[k] / 255.0 * (1 - a))), 0, 255);
                    }
                    o[3] = (uint8_t) std::clamp((int) std::lround(255.0 * (a + o[3] / 255.0 * (1 - a))), 0, 255);
                }
        }
    });
}

static int vr_overlay_get_image(mlt_frame a_frame, uint8_t **image, mlt_image_format *format, int *width, int *height,
                                int writable)
{
    (void) writable;
    mlt_frame b_frame = mlt_frame_pop_frame(a_frame);
    mlt_transition t = (mlt_transition) mlt_frame_pop_service(a_frame);
    mlt_properties p = MLT_TRANSITION_PROPERTIES(t);
    mlt_position pos = mlt_transition_get_position(t, a_frame), len = mlt_transition_get_length(t);
    *format = mlt_image_rgba;
    int err = mlt_frame_get_image(a_frame, image, format, width, height, 1);
    if (err || !*image) return err;
    uint8_t *bimg = NULL;
    mlt_image_format bfmt = mlt_image_rgba;
    int bw = *width, bh = *height;
    if (mlt_frame_get_image(b_frame, &bimg, &bfmt, &bw, &bh, 0) || !bimg || bfmt != mlt_image_rgba || bw <= 0 || bh <= 0)
        return 0;  // nothing to place
    mlt_service_lock(MLT_TRANSITION_SERVICE(t));
    OverlaySetup S = read_setup(p, pos, len, *width, *height);
    mlt_service_unlock(MLT_TRANSITION_SERVICE(t));
    if (S.lp.opacity <= 0) return 0;
    render_overlay(S, bimg, bw, bh, *image, true);  // writable A image, composited in place
    return 0;
}

// The FILTER form, for Shotcut: added to the overlay clip on an upper track. Its frame is the texture; the output is
// a VR frame of the same size with only the placed layer (straight alpha), which the track blending lays over the
// VR footage below.
static int vr_overlay_filter_get_image(mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width,
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
    OverlaySetup S = read_setup(p, pos, len, *width, *height);
    mlt_service_unlock(MLT_FILTER_SERVICE(filter));
    int size = *width * *height * 4;
    uint8_t *out = (uint8_t *) mlt_pool_alloc(size);
    std::memset(out, 0, size);
    if (S.lp.opacity > 0) render_overlay(S, *image, *width, *height, out, false);
    mlt_frame_set_image(frame, out, size, mlt_pool_release);
    *image = out;
    return 0;
}

static mlt_frame vr_overlay_process(mlt_transition transition, mlt_frame a_frame, mlt_frame b_frame)
{
    mlt_frame_push_service(a_frame, transition);
    mlt_frame_push_frame(a_frame, b_frame);
    mlt_frame_push_get_image(a_frame, vr_overlay_get_image);
    return a_frame;
}

static void overlay_defaults(mlt_properties p)
{
    mlt_properties_set(p, "packing", "sbs");
    mlt_properties_set_double(p, "yaw", 0);
    mlt_properties_set_double(p, "pitch", 0);
    mlt_properties_set_double(p, "distance", 2);
    mlt_properties_set_double(p, "width", 1);
    mlt_properties_set_double(p, "height", 0);
    mlt_properties_set(p, "size_mode", "metric");
    mlt_properties_set(p, "facing", "viewer");
    mlt_properties_set(p, "shape", "flat");
    mlt_properties_set(p, "rig", "auto");
    mlt_properties_set_double(p, "baseline", 0.063);
    mlt_properties_set_double(p, "opacity", 1);
    mlt_properties_set_int(p, "ss", 1);
}

static mlt_frame vr_overlay_filter_process(mlt_filter filter, mlt_frame frame)
{
    mlt_frame_push_service(frame, filter);
    mlt_frame_push_get_image(frame, vr_overlay_filter_get_image);
    return frame;
}

extern "C" mlt_filter filter_vr_overlay_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg)
{
    (void) profile; (void) type; (void) id; (void) arg;
    mlt_filter filter = mlt_filter_new();
    if (filter) {
        overlay_defaults(MLT_FILTER_PROPERTIES(filter));
        filter->process = vr_overlay_filter_process;
    }
    return filter;
}

extern "C" mlt_transition transition_vr_overlay_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg)
{
    (void) profile; (void) type; (void) id; (void) arg;
    mlt_transition t = mlt_transition_new();
    if (t) {
        overlay_defaults(MLT_TRANSITION_PROPERTIES(t));
        t->process = vr_overlay_process;
    }
    return t;
}
