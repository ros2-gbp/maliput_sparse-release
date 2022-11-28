##############################################################################
# osm
##############################################################################

if(BUILD_TESTING)

add_library(test_utilities
  maliput_sparse_types_compare.cc
)
add_library(maliput_sparse::test_utilities ALIAS test_utilities)
set_target_properties(test_utilities
  PROPERTIES
    OUTPUT_NAME maliput_sparse_test_utilities
)

target_include_directories(
  test_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(test_utilities
   PRIVATE
    maliput::test_utilities
    maliput_sparse::geometry
)

install(TARGETS test_utilities
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

endif()
