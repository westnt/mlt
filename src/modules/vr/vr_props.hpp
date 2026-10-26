// Shared helpers: read the VR clip layout from MLT properties (the "VR clip" property of FEATURES.md §1).
#pragma once
#include <framework/mlt.h>
#include <cstring>
#include "vrcore.hpp"  // port/core/vrcore.hpp (copied into the module dir when built inside MLT)

// Properties (all filters): packing = mono|sbs|tb (default sbs), lon_min/lon_max/lat_min/lat_max in degrees
// (default VR180: -90 90 -90 90), swap_eyes 0|1. Frame width/height give the eye size.
inline vrcore::Layout vr_layout_from(mlt_properties p, int width, int height)
{
    vrcore::Layout L;
    const char *pk = mlt_properties_get(p, "packing");
    L.packing = !pk || !strcmp(pk, "sbs") ? vrcore::Packing::SBS
                : !strcmp(pk, "tb")       ? vrcore::Packing::TB
                                          : vrcore::Packing::Mono;
    L.eye_width = L.packing == vrcore::Packing::SBS ? width / 2 : width;
    L.eye_height = L.packing == vrcore::Packing::TB ? height / 2 : height;
    auto get = [&](const char *k, double d) { return mlt_properties_get(p, k) ? mlt_properties_get_double(p, k) : d; };
    L.lon0 = get("lon_min", -90);
    L.lon1 = get("lon_max", 90);
    L.lat0 = get("lat_min", -90);
    L.lat1 = get("lat_max", 90);
    L.swap_eyes = mlt_properties_get_int(p, "swap_eyes");
    return L;
}
