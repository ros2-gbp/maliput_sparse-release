##############################################################################
# parser
##############################################################################
add_library(parser
  connection.cc
  lane.cc
  validator.cc
)

add_library(maliput_sparse::parser ALIAS parser)
set_target_properties(parser
  PROPERTIES
    OUTPUT_NAME maliput_sparse_parser
)

target_include_directories(
    parser
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(parser
   PRIVATE
    maliput::common
    maliput_sparse::geometry
)

install(TARGETS parser
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
