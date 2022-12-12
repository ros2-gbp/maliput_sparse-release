##############################################################################
# Sources
##############################################################################

set(LOADER_SOURCES
  builder_configuration.cc
  road_geometry_loader.cc
  road_network_loader.cc
)

add_library(loader ${LOADER_SOURCES})

add_library(maliput_sparse::loader ALIAS loader)

set_target_properties(loader
  PROPERTIES
    OUTPUT_NAME maliput_sparse_loader
)

target_include_directories(loader
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(loader
  PUBLIC
  maliput::api
  maliput::base
  maliput::common
  maliput_sparse::builder
)

##############################################################################
# Export
##############################################################################

install(TARGETS loader
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
