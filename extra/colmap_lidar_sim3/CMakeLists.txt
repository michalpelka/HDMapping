cmake_minimum_required(VERSION 4.0.0)

project(colmap_lidar_sim3)

# Experimental, kept off main: registers a COLMAP model to a lidar LAZ map with a global Sim(3) ICP
# and a per-camera Sim(3) refinement (fixes SfM "banana" drift). Needs Ceres 2.2+ (Manifold API).
# Built only with -DBUILD_COLMAP_LIDAR_SIM3=ON.
find_package(Ceres REQUIRED)

# GUI-free part, shared by the app and its tests.
add_library(colmap_lidar_sim3_core STATIC
    camera_projection.cpp
    colmap_model.cpp
    lidar_plane_map.cpp
    lidar_point_map.cpp
    photometric_refine.cpp
    sim3_refine.cpp
)

target_include_directories(colmap_lidar_sim3_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(colmap_lidar_sim3_core
    PUBLIC unordered_dense::unordered_dense
    PRIVATE Ceres::ceres ${PLATFORM_MISCELLANEOUS_LIBS}
)

if (MSVC)
    target_compile_definitions(colmap_lidar_sim3_core PUBLIC _USE_MATH_DEFINES)
endif()

set(SOURCES
    colmap_lidar_sim3.cpp
    app_shell.cpp
    camera_view.cpp
)

add_executable(colmap_lidar_sim3 ${SOURCES})

target_compile_definitions(colmap_lidar_sim3 PRIVATE -DWITH_GUI=1)

target_include_directories(
  colmap_lidar_sim3
  PRIVATE ${REPOSITORY_DIRECTORY}/core/include
          ${THIRDPARTY_DIRECTORY}
          ${THIRDPARTY_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/imgui
          ${THIRDPARTY_DIRECTORY}/imgui/backends
          ${THIRDPARTY_DIRECTORY}/ImGuizmo
          ${THIRDPARTY_DIRECTORY}/json/include
          ${THIRDPARTY_DIRECTORY}/portable-file-dialogs-master
          ${THIRDPARTY_DIRECTORY}/glew-cmake/include
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes
          ${THIRDPARTY_DIRECTORY}/freeglut/include
          ${LASZIP_INCLUDE_DIR}/LASzip/include)

target_link_libraries(
  colmap_lidar_sim3
  colmap_lidar_sim3_core
  core_raylib
  raylib_widgets
  imgui_raylib
  rlimgui
  imguizmo_raylib
  ${PLATFORM_LASZIP_LIB}
  ${PLATFORM_MISCELLANEOUS_LIBS}
  ${OpenCV_LIBS}
  spdlog::spdlog)

if(WIN32)
  add_custom_command(
    TARGET colmap_lidar_sim3
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy
      $<TARGET_RUNTIME_DLLS:colmap_lidar_sim3>
      $<TARGET_FILE_DIR:colmap_lidar_sim3>
    COMMAND_EXPAND_LISTS)
endif()

if (MSVC)
    target_compile_options(colmap_lidar_sim3 PRIVATE /bigobj)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
