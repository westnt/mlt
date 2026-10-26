/*
 * factory.c -- the "vr" MLT module: VR (180/360, mono/stereo) aware filters.
 * Copyright (C) 2026 the vredit authors. GPL-2.0-or-later.
 * Reference implementation and tests: vredit (Python), port/tests.
 */
#include <framework/mlt.h>
#include <limits.h>
#include <string.h>

extern mlt_filter filter_vr_transform_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);
extern mlt_filter filter_vr_mask_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);
extern mlt_filter filter_vr_eyealign_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);
extern mlt_filter filter_vr_colormatch_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);
extern mlt_filter filter_vr_flat_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);
extern mlt_filter filter_vr_anaglyph_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);
extern mlt_filter filter_vr_convergence_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);
extern mlt_filter filter_vr_overlay_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);
extern mlt_transition transition_vr_overlay_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg);

static mlt_properties metadata(mlt_service_type type, const char *id, void *data)
{
    char file[PATH_MAX];
    snprintf(file, PATH_MAX, "%s/vr/%s", mlt_environment("MLT_DATA"), (char *) data);
    return mlt_properties_parse_yaml(file);
}

MLT_REPOSITORY
{
    MLT_REGISTER(mlt_service_filter_type, "vr_transform", filter_vr_transform_init);
    MLT_REGISTER_METADATA(mlt_service_filter_type, "vr_transform", metadata, "filter_vr_transform.yml");
    MLT_REGISTER(mlt_service_filter_type, "vr_mask", filter_vr_mask_init);
    MLT_REGISTER_METADATA(mlt_service_filter_type, "vr_mask", metadata, "filter_vr_mask.yml");
    MLT_REGISTER(mlt_service_filter_type, "vr_eyealign", filter_vr_eyealign_init);
    MLT_REGISTER_METADATA(mlt_service_filter_type, "vr_eyealign", metadata, "filter_vr_eyealign.yml");
    MLT_REGISTER(mlt_service_filter_type, "vr_colormatch", filter_vr_colormatch_init);
    MLT_REGISTER_METADATA(mlt_service_filter_type, "vr_colormatch", metadata, "filter_vr_colormatch.yml");
    MLT_REGISTER(mlt_service_filter_type, "vr_flat", filter_vr_flat_init);
    MLT_REGISTER_METADATA(mlt_service_filter_type, "vr_flat", metadata, "filter_vr_flat.yml");
    MLT_REGISTER(mlt_service_filter_type, "vr_anaglyph", filter_vr_anaglyph_init);
    MLT_REGISTER_METADATA(mlt_service_filter_type, "vr_anaglyph", metadata, "filter_vr_anaglyph.yml");
    MLT_REGISTER(mlt_service_filter_type, "vr_convergence", filter_vr_convergence_init);
    MLT_REGISTER_METADATA(mlt_service_filter_type, "vr_convergence", metadata, "filter_vr_convergence.yml");
    MLT_REGISTER(mlt_service_filter_type, "vr_overlay", filter_vr_overlay_init);
    MLT_REGISTER_METADATA(mlt_service_filter_type, "vr_overlay", metadata, "filter_vr_overlay_filter.yml");
    MLT_REGISTER(mlt_service_transition_type, "vr_overlay", transition_vr_overlay_init);
    MLT_REGISTER_METADATA(mlt_service_transition_type, "vr_overlay", metadata, "filter_vr_overlay.yml");
}
