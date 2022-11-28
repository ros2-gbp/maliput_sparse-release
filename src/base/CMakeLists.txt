##############################################################################
# Sources
##############################################################################

set(BASE_SOURCES
  lane.cc
  road_geometry.cc
)

add_library(base ${BASE_SOURCES})

add_library(maliput_sparse::base ALIAS base)

set_target_properties(base
  PROPERTIES
    OUTPUT_NAME maliput_sparse_base
)

target_include_directories(base
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(base
  PUBLIC
  maliput::api
  maliput::common
  maliput::geometry_base
  maliput::math
)

##############################################################################
# Export
##############################################################################

include(CMakePackageConfigHelpers)

install(
  TARGETS base
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
