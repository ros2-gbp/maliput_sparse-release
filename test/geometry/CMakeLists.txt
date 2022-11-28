ament_add_gtest(lane_geometry_test lane_geometry_test.cc)
ament_add_gtest(line_string_test line_string_test.cc)
ament_add_gtest(geometry_test utility/geometry_test.cc)

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
          maliput::math
          maliput::test_utilities
          maliput_sparse::geometry
      )

    endif()
endmacro()

add_dependencies_to_test(lane_geometry_test)
add_dependencies_to_test(line_string_test)
add_dependencies_to_test(geometry_test)
