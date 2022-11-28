##############################################################################
# Sources
##############################################################################

set(BUILDER_SOURCES
  builder.cc
)

add_library(builder ${BUILDER_SOURCES})

add_library(maliput_sparse::builder ALIAS builder)

set_target_properties(builder
  PROPERTIES
    OUTPUT_NAME maliput_sparse_builder
)

target_include_directories(builder
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(builder
  PUBLIC
  maliput::api
  maliput::common
  maliput::geometry_base
  maliput::math
  maliput_sparse::base
  maliput_sparse::geometry
)

##############################################################################
# Export
##############################################################################

install(TARGETS builder
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
