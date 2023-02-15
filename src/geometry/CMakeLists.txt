##############################################################################
# Sources
##############################################################################

set(GEOMETRY_SOURCES
  line_string.cc
  lane_geometry.cc
  utility/geometry.cc
)

add_library(geometry ${GEOMETRY_SOURCES})

add_library(maliput_sparse::geometry ALIAS geometry)

set_target_properties(geometry
  PROPERTIES
    OUTPUT_NAME maliput_sparse_geometry
)

target_include_directories(geometry
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(geometry
  PUBLIC
    maliput::common
    maliput::math
)

##############################################################################
# Export
##############################################################################

include(CMakePackageConfigHelpers)

install(
  TARGETS geometry
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
