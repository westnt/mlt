// Boilerplate shared by the vr_* filters: fetch RGBA, run a per-pixel kernel over row slices, set the new image.
#pragma once
#include "vr_props.hpp"
#include <framework/mlt_slices.h>
#include <functional>

using RowKernel = std::function<void(int row_begin, int row_end)>;

struct slice_job
{
    RowKernel *kernel;
    int rows;
};

inline int vr_slice_proc(int id, int index, int jobs, void *cookie)
{
    (void) id;
    slice_job *j = (slice_job *) cookie;
    int start, n = mlt_slices_size_slice(jobs, index, j->rows, &start);
    (*j->kernel)(start, start + n);
    return 0;
}

inline void vr_run_rows(int rows, RowKernel k)
{
    slice_job j{&k, rows};
    mlt_slices_run_normal(0, vr_slice_proc, &j);
}

inline double vr_anim(mlt_properties p, const char *name, mlt_position pos, mlt_position len, double dflt)
{
    return mlt_properties_get(p, name) ? mlt_properties_anim_get_double(p, name, pos, len) : dflt;
}

#define VR_FILTER_BOILERPLATE(NAME, GET_IMAGE, DEFAULTS)                                                              \
    static mlt_frame NAME##_process(mlt_filter filter, mlt_frame frame)                                             \
    {                                                                                                                 \
        mlt_frame_push_service(frame, filter);                                                                        \
        mlt_frame_push_get_image(frame, GET_IMAGE);                                                                   \
        return frame;                                                                                                 \
    }                                                                                                                 \
    extern "C" mlt_filter filter_##NAME##_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg) \
    {                                                                                                                 \
        (void) profile; (void) type; (void) id; (void) arg;                                                           \
        mlt_filter filter = mlt_filter_new();                                                                         \
        if (filter) {                                                                                                 \
            mlt_properties p = MLT_FILTER_PROPERTIES(filter);                                                         \
            mlt_properties_set(p, "packing", "sbs");                                                                  \
            DEFAULTS;                                                                                                 \
            filter->process = NAME##_process;                                                                         \
        }                                                                                                             \
        return filter;                                                                                                \
    }
