add_library(mltvr MODULE
    factory.c
    filter_vr_anaglyph.cpp
    filter_vr_colormatch.cpp
    filter_vr_eyealign.cpp
    filter_vr_flat.cpp
    filter_vr_mask.cpp
    filter_vr_transform.cpp
    filter_vr_convergence.cpp
    filter_vr_overlay.cpp
)

file(GLOB YML "*.yml")
add_custom_target(Other_vr_Files SOURCES
  ${YML}
)

target_compile_options(mltvr PRIVATE ${MLT_COMPILE_OPTIONS})
target_compile_features(mltvr PRIVATE cxx_std_17)
target_link_libraries(mltvr PRIVATE mlt)

set_target_properties(mltvr PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${MLT_MODULE_OUTPUT_DIRECTORY}")

install(TARGETS mltvr LIBRARY DESTINATION ${MLT_INSTALL_MODULE_DIR})

install(FILES ${YML} DESTINATION ${MLT_INSTALL_DATA_DIR}/vr)
