ament_add_gtest(lane_test lane_test.cc)
ament_add_gtest(road_geometry_test road_geometry_test.cc)

macro(add_dependencies_to_test target)
if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/include
          ${PROJECT_SOURCE_DIR}/src
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::api
          maliput::common
          maliput::geometry_base
          maliput::math
          maliput::test_utilities
          maliput_sparse::base
          maliput_sparse::builder
          maliput_sparse::geometry
      )

    endif()
endmacro()

add_dependencies_to_test(lane_test)
add_dependencies_to_test(road_geometry_test)
