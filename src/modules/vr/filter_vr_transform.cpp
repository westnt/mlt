/*
 * filter_vr_transform.cpp -- rotate the whole view of a VR clip, the same rotation for every eye.
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.
 *
 * Unlike "360: Transform" (bigsh0t), which treats the frame as one mono 360 image, this reads the VR layout
 * (side-by-side / top-bottom, 180 or 360) and rotates each eye identically, so stereo is preserved.
 * Angles: same meaning as bigsh0t (+yaw = look right, +pitch = look up, +roll = clockwise), pixel centers at +0.5.
 * Reference: vredit/view/transform.py; checked pixel by pixel in port/tests/test_port_mlt.py.
 */
#include "vr_props.hpp"
#include <framework/mlt_slices.h>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <vector>

// The rotation's per-pixel source positions, kept while the rotation, layout and frame size stay the same (a
// fixed, un-animated rotation then costs only a bilinear sample per pixel instead of the trigonometry).
struct transform_map
{
    double key[10];
    std::vector<float> uv;
};

struct map_cache
{
    std::mutex lock;
    std::shared_ptr<transform_map> map;
};

static void map_cache_close(void *p)
{
    delete (map_cache *) p;
}

struct build_job
{
    vrcore::Layout layout;
    vrcore::Mat3 R;
    float *uv;
};

static int build_proc(int id, int index, int jobs, void *cookie)
{
    (void) id;
    build_job *j = (build_job *) cookie;
    int start, n = mlt_slices_size_slice(jobs, index, j->layout.eye_height, &start);
    vrcore::build_transform_map(j->layout, j->R, j->uv, start, start + n);
    return 0;
}

struct job
{
    const uint8_t *src;
    uint8_t *dst;
    vrcore::Layout layout;
    const float *uv;
    bool transparent;
};

static int slice_proc(int id, int index, int jobs, void *cookie)
{
    (void) id;
    job *j = (job *) cookie;
    int start, n = mlt_slices_size_slice(jobs, index, j->layout.eye_height, &start);
    vrcore::apply_transform_map(j->src, j->dst, j->layout, j->uv, start, start + n, j->transparent);
    return 0;
}

static std::shared_ptr<transform_map> get_map(mlt_properties p, const vrcore::Layout &L, double yaw, double pitch,
                                              double roll)
{
    map_cache *cache = (map_cache *) mlt_properties_get_data(p, "_vr_transform_map", NULL);
    if (!cache) {
        cache = new map_cache;
        mlt_properties_set_data(p, "_vr_transform_map", cache, 0, map_cache_close, NULL);
    }
    vrcore::Equirect e = L.eye_projection();
    const double key[10] = {(double) L.eye_width, (double) L.eye_height, (double) (int) L.packing,
                            e.lon0, e.lon1, e.lat0, e.lat1, yaw, pitch, roll};
    std::lock_guard<std::mutex> g(cache->lock);
    if (cache->map && !std::memcmp(cache->map->key, key, sizeof key))
        return cache->map;
    auto m = std::make_shared<transform_map>();
    std::memcpy(m->key, key, sizeof key);
    m->uv.resize(2 * (size_t) L.eye_width * L.eye_height);
    build_job b{L, vrcore::ypr_to_matrix(yaw, pitch, roll), m->uv.data()};
    mlt_slices_run_normal(0, build_proc, &b);
    cache->map = m;  // an animated rotation replaces it every frame: no worse than computing it directly
    return m;
}

static int filter_get_image(mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width, int *height,
                            int writable)
{
    mlt_filter filter = (mlt_filter) mlt_frame_pop_service(frame);
    mlt_properties p = MLT_FILTER_PROPERTIES(filter);
    mlt_position pos = mlt_filter_get_position(filter, frame);
    mlt_position len = mlt_filter_get_length2(filter, frame);
    *format = mlt_image_rgba;
    int err = mlt_frame_get_image(frame, image, format, width, height, 0);
    if (err || !*image) return err;
    mlt_service_lock(MLT_FILTER_SERVICE(filter));
    double yaw = mlt_properties_anim_get_double(p, "yaw", pos, len);
    double pitch = mlt_properties_anim_get_double(p, "pitch", pos, len);
    double roll = mlt_properties_anim_get_double(p, "roll", pos, len);
    vrcore::Layout L = vr_layout_from(p, *width, *height);
    bool transparent = mlt_properties_get_int(p, "transparent") != 0;
    mlt_service_unlock(MLT_FILTER_SERVICE(filter));
    if (yaw == 0 && pitch == 0 && roll == 0) return 0;
    std::shared_ptr<transform_map> map = get_map(p, L, yaw, pitch, roll);
    int size = *width * *height * 4;  // RGBA8 (mlt_image_format_size is deprecated)
    uint8_t *out = (uint8_t *) mlt_pool_alloc(size);
    job j{*image, out, L, map->uv.data(), transparent};
    mlt_slices_run_normal(0, slice_proc, &j);
    mlt_frame_set_image(frame, out, size, mlt_pool_release);
    *image = out;
    return 0;
}

static mlt_frame filter_process(mlt_filter filter, mlt_frame frame)
{
    mlt_frame_push_service(frame, filter);
    mlt_frame_push_get_image(frame, filter_get_image);
    return frame;
}

extern "C" mlt_filter filter_vr_transform_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg)
{
    (void) profile; (void) type; (void) id; (void) arg;
    mlt_filter filter = mlt_filter_new();
    if (filter) {
        mlt_properties p = MLT_FILTER_PROPERTIES(filter);
        mlt_properties_set(p, "packing", "sbs");
        mlt_properties_set_double(p, "yaw", 0);
        mlt_properties_set_double(p, "pitch", 0);
        mlt_properties_set_double(p, "roll", 0);
        mlt_properties_set_int(p, "transparent", 1);  // outside the picture: see-through
        filter->process = filter_process;
    }
    return filter;
}
