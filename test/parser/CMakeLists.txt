ament_add_gtest(lane_parser_test lane_test.cc)
ament_add_gtest(segment_parser_test segment_test.cc)
ament_add_gtest(validator_test validator_test.cc)

macro(add_dependencies_to_test target)
if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::common
          maliput_sparse::geometry
          maliput_sparse::parser
      )

    endif()
endmacro()

add_dependencies_to_test(validator_test)
add_dependencies_to_test(lane_parser_test)
add_dependencies_to_test(segment_parser_test)
