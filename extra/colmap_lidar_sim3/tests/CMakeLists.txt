cmake_minimum_required(VERSION 4.0.0)

project(colmap_lidar_sim3_tests)

# doctest, like shared/tests; test_colmap_model.cpp owns main().
add_executable(colmap_lidar_sim3_tests
    test_colmap_model.cpp
    test_camera_projection.cpp
    test_lidar_plane_map.cpp
    test_sim3_refine.cpp
)

target_link_libraries(colmap_lidar_sim3_tests PRIVATE colmap_lidar_sim3_core)

target_include_directories(colmap_lidar_sim3_tests PRIVATE
    ${THIRDPARTY_DIRECTORY}/doctest
)

include(CTest)
add_test(NAME colmap_lidar_sim3_tests COMMAND colmap_lidar_sim3_tests)
