# cmake/dependencies.cmake
# Extract 3rdparty and external library configuration
# Included from root CMakeLists.txt

include_guard()

message(STATUS "==== Configuring Dependencies ====")

# ============================================================================
# Graphics Libraries (Bundled or System)
# ============================================================================

# Eigen3 - Linear algebra library (bundled)
add_subdirectory(${THIRDPARTY_DIRECTORY}/eigen)
set(EIGEN3_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/eigen)
message(STATUS "Using bundled Eigen3 : ${EIGEN3_INCLUDE_DIR}")

# FreeGLUT - OpenGL utilities
if (BUILD_WITH_BUNDLED_FREEGLUT)
    set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/freeglut)
    set(FREEGLUT_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/freeglut/include)
    set(FREEGLUT_LIBRARY freeglut)
    
    # Suppress unused variable warnings from FreeGlut
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(freeglut PRIVATE -Wno-unused-const-variable)
    endif()
    message(STATUS "Using bundled FreeGlut")
else()
    find_package(GLUT REQUIRED)
    set(FREEGLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
    set(FREEGLUT_LIBRARY ${GLUT_glut_LIBRARY})
    message(STATUS "FreeGlut include dir: ${FREEGLUT_INCLUDE_DIR}")
endif()

# GLEW - OpenGL extension loader
if (BUILD_WITH_BUNDLED_GLEW)
    if(APPLE)
        # On modern macOS AGL framework is deprecated/removed.
        # Point AGL_LIBRARY to OpenGL to trick the submodule into linking against a valid framework
        find_library(AGL_LIBRARY OpenGL)
    endif()

    add_subdirectory(${THIRDPARTY_DIRECTORY}/glew-cmake)
    set(GLEW_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/glew-cmake/include)
    set(GLEW_LIBRARY libglew_static)
    message(STATUS "Using bundled GLEW")
else()
    find_package(GLEW REQUIRED)
    set(GLEW_INCLUDE_DIR ${GLEW_INCLUDE_DIRS})
    set(GLEW_LIBRARY ${GLEW_LIBRARIES})
    message(STATUS "GLEW include dir: ${GLEW_INCLUDE_DIR}, GLEW library: ${GLEW_LIBRARY}")
endif()

# ============================================================================
# Point Cloud & LiDAR Libraries
# ============================================================================

# LASzip - LAS file compression
if (BUILD_WITH_BUNDLED_LIBLASZIP)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/LASzip)
    set(LASZIP_INCLUDE_DIR ${THIRDPARTY_DIRECTORY})
    set(LASZIP_LIBRARY laszip)
    message(STATUS "Using bundled LASzip")
    
    if(APPLE)
        target_compile_definitions(laszip PUBLIC off64_t=off_t)
    endif()
else()
    find_package(LASzip REQUIRED)
    set(LASZIP_INCLUDE_DIR ${LASZIP_INCLUDE_DIR})
    set(LASZIP_LIBRARY ${LASZIP_LIBRARY})
    set(PLATFORM_LASZIP_LIB ${LASZIP_LIBRARY})
    message(STATUS "LASzip include dir: ${LASZIP_INCLUDE_DIR}, LASzip library: ${LASZIP_LIBRARY}")
endif()

# E57 - Apache Xerces-C + libE57Format, both vendored as git submodules and
# built from source (no system packages). Xerces-C provides the XML parser
# libE57Format needs; cmake/FindXercesC.cmake redirects libE57Format's
# find_package(XercesC) to the in-tree `xerces-c` target defined here.
#
# NOTE: xerces-c's bundled tests/samples are kept out of CTest -- see the
# add_subdirectory() override further down. libE57Format's own tests are
# disabled via E57_BUILD_TEST=OFF.
#
# xerces-c/cmake/XercesDLL.cmake unconditionally runs
#   set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")
# With no prior cache entry that creates BUILD_SHARED_LIBS=ON in the cache and
# flips EVERY dependency added afterwards (vqf, Fusion, glad, plycpp, ...) to
# a DLL with no exported symbols -- on Windows that means no import .lib and a
# project-wide link failure. Pin a cache entry to OFF up front so that set()
# becomes a no-op, then remove it so nothing else is affected. HDMapping builds
# all dependencies statically, so an existing BUILD_SHARED_LIBS is not
# expected; save and restore it anyway.
if(DEFINED CACHE{BUILD_SHARED_LIBS})
    set(_hd_bsl_prev "$CACHE{BUILD_SHARED_LIBS}")
    set(_hd_bsl_restore ON)
else()
    set(_hd_bsl_restore OFF)
endif()
set(BUILD_SHARED_LIBS OFF CACHE BOOL "HDMapping links Xerces-C + libE57Format statically" FORCE)

# Keep xerces-c's bundled `tests` and `samples` subdirectories out of the
# configure entirely: they unconditionally call enable_testing() + add_test()
# for ~80 sample-driven cases whose executables we never build
# (EXCLUDE_FROM_ALL), which then fail `ctest` in CI. Override add_subdirectory()
# to drop those two leaves while the _hd_skip_test_subdirs guard is set; the
# builtin stays reachable as _add_subdirectory() and the guard is cleared right
# after so every later add_subdirectory() (libE57Format, core, apps, HDMapping's
# own tests) behaves normally.
set(_hd_skip_test_subdirs ON)
macro(add_subdirectory _hd_dir)
    get_filename_component(_hd_leaf "${_hd_dir}" NAME)
    if(_hd_skip_test_subdirs AND (_hd_leaf STREQUAL "tests" OR _hd_leaf STREQUAL "samples"))
        message(STATUS "Skipping xerces-c '${_hd_leaf}' subdirectory (E57 tests disabled)")
    else()
        _add_subdirectory("${_hd_dir}" ${ARGN})
    endif()
endmacro()

# Xerces-C transcoder/netaccessor/message-loader defaults are the platform
# native, dependency-free choices (macOS: macosunicodeconverter/cfurl,
# Windows: windows/winsock, Linux: iconv/socket) -- no ICU, no libcurl.
add_subdirectory(${THIRDPARTY_DIRECTORY}/xerces-c ${CMAKE_BINARY_DIR}/3rdparty/xerces-c EXCLUDE_FROM_ALL)

set(_hd_skip_test_subdirs OFF)

# libE57Format: static lib target `E57Format`, headers at libE57Format/include.
set(E57_BUILD_TEST OFF CACHE BOOL "" FORCE)
set(E57_RELEASE_LTO OFF CACHE BOOL "" FORCE)   # don't force LTO into the wider build
if(WIN32)
    set(USING_STATIC_XERCES ON CACHE BOOL "" FORCE)   # adds XERCES_STATIC_LIBRARY define
endif()
add_subdirectory(${THIRDPARTY_DIRECTORY}/libE57Format ${CMAKE_BINARY_DIR}/3rdparty/libE57Format EXCLUDE_FROM_ALL)
set(LIBE57FORMAT_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/libE57Format/include)

if(_hd_bsl_restore)
    set(BUILD_SHARED_LIBS "${_hd_bsl_prev}" CACHE BOOL "Build shared libraries" FORCE)
else()
    unset(BUILD_SHARED_LIBS CACHE)
endif()
message(STATUS "Using bundled Xerces-C + libE57Format (static) for E57 support")

# ============================================================================
# Computer Vision & Geospatial Libraries (Pre-downloaded)
# ============================================================================

# OpenCV - Computer vision library
include(${THIRDPARTY_DIRECTORY_BINARY}/OpenCV/CMakeLists.txt)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}, OpenCV libs: ${OpenCV_LIBS}")

# fbow - Fast Bag-of-Words image retrieval (needs OpenCV, added above)
set(BUILD_UTILS OFF CACHE BOOL "" FORCE)
# fbow's own CMakeLists defaults USE_SSE3/USE_AVX to ON unconditionally (only
# guards -O2 vs -O3 by CMAKE_SYSTEM_PROCESSOR, not the x86 vector flags), which
# fails on arm64 (e.g. Apple Silicon) where -msse3/-mavx aren't valid clang flags.
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|i686|x86)$")
    set(USE_SSE3 OFF CACHE BOOL "" FORCE)
    set(USE_AVX OFF CACHE BOOL "" FORCE)
    # fbow's sources include <immintrin.h> and use cpuid unless built for
    # Android; the patch takes those portable paths on any non-x86 CPU.
    # Applied once to the submodule's working tree (skipped if already applied).
    find_package(Git REQUIRED)
    set(FBOW_PATCH ${THIRDPARTY_DIRECTORY}/patches/fbow_non_x86.patch)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} apply --reverse --check ${FBOW_PATCH}
        WORKING_DIRECTORY ${THIRDPARTY_DIRECTORY}/fbow
        RESULT_VARIABLE FBOW_PATCH_APPLIED
        OUTPUT_QUIET ERROR_QUIET)
    if(NOT FBOW_PATCH_APPLIED EQUAL 0)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} apply ${FBOW_PATCH}
            WORKING_DIRECTORY ${THIRDPARTY_DIRECTORY}/fbow
            RESULT_VARIABLE FBOW_PATCH_RESULT)
        if(NOT FBOW_PATCH_RESULT EQUAL 0)
            message(FATAL_ERROR "Failed to apply ${FBOW_PATCH} to ${THIRDPARTY_DIRECTORY}/fbow")
        endif()
        message(STATUS "Applied ${FBOW_PATCH}")
    endif()
endif()
# fbow's CMakeLists leaks global cache entries that would break every target
# added after it (same trap as Xerces above):
#   option(BUILD_SHARED_LIBS ... ON) -- flips raylib, glad, ... to shared libs
#     that hide their bundled glfw/glad symbols (undefined glfw*/glad_gl* at link);
#   WIN32: EXECUTABLE/LIBRARY/RUNTIME_OUTPUT_PATH -- redirect every later
#     target's output (e.g. vqf.lib no longer under lib/Release).
# Pin BUILD_SHARED_LIBS=OFF so fbow builds static and the option() is a no-op,
# then restore/remove those cache entries once fbow is configured.
if(DEFINED CACHE{BUILD_SHARED_LIBS})
    set(_hd_bsl_prev "$CACHE{BUILD_SHARED_LIBS}")
    set(_hd_bsl_restore ON)
else()
    set(_hd_bsl_restore OFF)
endif()
set(BUILD_SHARED_LIBS OFF CACHE BOOL "HDMapping links fbow statically" FORCE)
set(FBOW_DEVINSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(${THIRDPARTY_DIRECTORY}/fbow)
if(_hd_bsl_restore)
    set(BUILD_SHARED_LIBS "${_hd_bsl_prev}" CACHE BOOL "Build shared libraries" FORCE)
else()
    unset(BUILD_SHARED_LIBS CACHE)
endif()
unset(EXECUTABLE_OUTPUT_PATH CACHE)
unset(LIBRARY_OUTPUT_PATH CACHE)
unset(RUNTIME_OUTPUT_PATH CACHE)
set(FBOW_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/fbow/src)
message(STATUS "Using bundled fbow from: ${THIRDPARTY_DIRECTORY}/fbow")

# PROJ - Cartographic projections library
include(${THIRDPARTY_DIRECTORY_BINARY}/Proj/CMakeLists.txt)
message(STATUS "PROJ include dir: ${PROJ_INCLUDE_DIR}, PROJ library: ${PROJ_LIBRARY}")
message(STATUS "PROJ implementation library: ${PROJ_IMPLIB}, PROJ database: ${PROJ_DB}")

# ============================================================================
# Threading & Performance Libraries
# ============================================================================

# oneTBB - Intel Threading Building Blocks (Bundled or System)
if (BUILD_WITH_BUNDLED_ONETBB)
    # Disable oneTBB components we don't need
    set(TBB_TEST OFF CACHE BOOL "" FORCE)
    set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(TBB_STRICT OFF CACHE BOOL "" FORCE)
    set(TBBMALLOC_BUILD OFF CACHE BOOL "" FORCE)
    set(TBB_CPF OFF CACHE BOOL "" FORCE)

    add_subdirectory(${THIRDPARTY_DIRECTORY}/oneTBB)
    message(STATUS "Using bundled oneTBB from: ${THIRDPARTY_DIRECTORY}/oneTBB")
else()
    find_package(TBB REQUIRED)
    message(STATUS "Found system TBB")
endif()

# ============================================================================
# Header-Only & Utility Libraries
# ============================================================================

# unordered_dense - Hash map library
add_subdirectory(${THIRDPARTY_DIRECTORY}/unordered_dense)
message(STATUS "Using bundled unordered_dense from: ${THIRDPARTY_DIRECTORY}/unordered_dense")

# spdlog - Logging library
set(SPDLOG_BUILD_PIC ON CACHE BOOL "Build spdlog with position-independent code")
add_subdirectory(${THIRDPARTY_DIRECTORY}/spdlog)
message(STATUS "Using bundled spdlog from: ${THIRDPARTY_DIRECTORY}/spdlog")

# UTL - Header-only utility library
add_subdirectory(${THIRDPARTY_DIRECTORY}/UTL)
message(STATUS "Using bundled UTL from: ${THIRDPARTY_DIRECTORY}/UTL")

# ============================================================================
# IMGui & Visualization Libraries
# ============================================================================

# Note: imgui, imguizmo, implot are configured separately in:
# cmake/imgui.cmake, cmake/imguizmo.cmake, cmake/implot.cmake

# VQF - Vector Quaternion Filter
add_subdirectory(${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp)
message(STATUS "Using bundled VQF from: ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp")

# Fusion - AHRS filter implementation
add_subdirectory(${THIRDPARTY_DIRECTORY}/Fusion/Fusion)
message(STATUS "Using bundled Fusion from: ${THIRDPARTY_DIRECTORY}/Fusion/Fusion")

# ============================================================================
# Profiler Backend - Optional
# ============================================================================

if(HDMAPPING_PROFILER STREQUAL "TRACY")
    include(FetchContent)
    FetchContent_Declare(tracy
        GIT_REPOSITORY https://github.com/wolfpld/tracy.git
        GIT_TAG        v0.13.1
        GIT_SHALLOW    TRUE
    )
    set(TRACY_ENABLE    ON CACHE BOOL "" FORCE)
    set(TRACY_ON_DEMAND ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(tracy)
    message(STATUS "HDMapping profiler: Tracy v0.13.1 (on-demand)")
elseif(HDMAPPING_PROFILER STREQUAL "UTL")
    message(STATUS "HDMapping profiler: UTL")
else()
    message(STATUS "HDMapping profiler: NONE (disabled)")
endif()

# ============================================================================
# Python Bindings - Optional
# ============================================================================

if(BUILD_WITH_PYBIND)
    message(STATUS "BUILD_WITH_PYBIND is enabled: fetching pybind11")
    include(FetchContent)
    set(pybind11_SOURCE_DIR "${CMAKE_BINARY_DIR}/3rdparty/pybind11-src")
    set(pybind11_BINARY_DIR "${CMAKE_BINARY_DIR}/3rdparty/pybind11-build")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11
        GIT_TAG        v2.12.1
        SOURCE_DIR     ${pybind11_SOURCE_DIR}
        BINARY_DIR     ${pybind11_BINARY_DIR}
    )
    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
        FetchContent_MakeAvailable(pybind11)
    endif()
    add_subdirectory(pybind)
else()
    message(STATUS "BUILD_WITH_PYBIND is disabled")
endif()

# ============================================================================
# External Coordinate Transformation Libraries
# ============================================================================

# Custom WGS84 projection utilities
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/plycpp)
message(STATUS "Using external library: plycpp")

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/wgs84_do_puwg92)
message(STATUS "Using external library: wgs84_do_puwg92")

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/WGS84toCartesian)
message(STATUS "Using external library: WGS84toCartesian")


# Compression libraries of the bag readers: LZ4 and Zstandard for mcap (MCAP chunks), bzip2 and LZ4 for
# rosbags/Ros1Bag (ROS1 bag chunks). Linux and macOS use the system packages (apt, Homebrew). Windows has no package
# manager for them, so they are built here as static libraries from the 3rdparty submodules.
if(WIN32)
    add_library(lz4_static STATIC
        ${THIRDPARTY_DIRECTORY}/lz4/lib/lz4.c
        ${THIRDPARTY_DIRECTORY}/lz4/lib/lz4hc.c
        ${THIRDPARTY_DIRECTORY}/lz4/lib/lz4frame.c
        ${THIRDPARTY_DIRECTORY}/lz4/lib/xxhash.c)
    target_include_directories(lz4_static PUBLIC ${THIRDPARTY_DIRECTORY}/lz4/lib)

    file(GLOB ZSTD_SOURCES
        ${THIRDPARTY_DIRECTORY}/zstd/lib/common/*.c
        ${THIRDPARTY_DIRECTORY}/zstd/lib/compress/*.c
        ${THIRDPARTY_DIRECTORY}/zstd/lib/decompress/*.c)
    add_library(zstd_static STATIC ${ZSTD_SOURCES})
    target_include_directories(zstd_static PUBLIC ${THIRDPARTY_DIRECTORY}/zstd/lib)
    target_compile_definitions(zstd_static PRIVATE ZSTD_DISABLE_ASM)

    add_library(bz2_static STATIC
        ${THIRDPARTY_DIRECTORY}/bzip2/blocksort.c
        ${THIRDPARTY_DIRECTORY}/bzip2/huffman.c
        ${THIRDPARTY_DIRECTORY}/bzip2/crctable.c
        ${THIRDPARTY_DIRECTORY}/bzip2/randtable.c
        ${THIRDPARTY_DIRECTORY}/bzip2/compress.c
        ${THIRDPARTY_DIRECTORY}/bzip2/decompress.c
        ${THIRDPARTY_DIRECTORY}/bzip2/bzlib.c)
    target_include_directories(bz2_static PUBLIC ${THIRDPARTY_DIRECTORY}/bzip2)
    # bzlib.h includes windows.h: keep its min/max macros out of the C++ that includes it.
    target_compile_definitions(bz2_static PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# mcap - support for ROS 2 bag export
add_subdirectory(${THIRDPARTY_DIRECTORY}/mcap)

# rosbags/PcapUdpReader reads zstd-compressed captures (.pcap.zst): the same libzstd as mcap.
add_library(hdmapping_zstd INTERFACE)
if(WIN32)
    target_link_libraries(hdmapping_zstd INTERFACE zstd_static)
else()
    target_link_libraries(hdmapping_zstd INTERFACE ${MCAP_ZSTD_LIBRARY})
    target_include_directories(hdmapping_zstd INTERFACE ${MCAP_ZSTD_INCLUDE_DIR})
endif()

# rosbags/Ros1Bag reads bz2 and lz4 chunks (lz4 comes with the mcap target).
add_library(ros1bag_compression INTERFACE)
if(WIN32)
    target_link_libraries(ros1bag_compression INTERFACE bz2_static)
else()
    find_package(BZip2 REQUIRED)
    target_link_libraries(ros1bag_compression INTERFACE BZip2::BZip2)
endif()

# Hesai LiDAR SDK 2.0 (hesai_pcap_to_laz): decodes Hesai point and IMU packets with the lidar's angle correction.
# Built without PTCS (TLS to a live sensor, which needs OpenSSL): reading captures needs no connection. The SDK
# supports Linux and Windows; on macOS cmake/hesai_macos_compat supplies <endian.h> and the Linux serial ioctls it
# names, force-included into the SDK and, through the hesai_sdk target, into the files that use it.
if(BUILD_WITH_CLI_TOOLS)
    set(WITH_PTCS_USE OFF CACHE BOOL "Hesai SDK: TLS connection to a live sensor (needs OpenSSL)" FORCE)
    set(DISENABLE_TEST_CC ON CACHE BOOL "Hesai SDK: skip its sample programs" FORCE)
    set(HESAI_SDK_DIRECTORY ${THIRDPARTY_DIRECTORY}/HesaiLidar_SDK_2.0)
    if(APPLE)
        set(HESAI_MACOS_COMPAT ${CMAKE_SOURCE_DIR}/cmake/hesai_macos_compat)
        set(HESAI_MACOS_FLAGS -I${HESAI_MACOS_COMPAT} -include ${HESAI_MACOS_COMPAT}/hesai_macos_compat.h)
    endif()
    get_directory_property(_hesai_saved_compile_options COMPILE_OPTIONS)
    add_compile_options(${HESAI_MACOS_FLAGS})
    add_subdirectory(${HESAI_SDK_DIRECTORY} ${CMAKE_BINARY_DIR}/3rdparty/HesaiLidar_SDK_2.0 EXCLUDE_FROM_ALL)
    set_directory_properties(PROPERTIES COMPILE_OPTIONS "${_hesai_saved_compile_options}")

    add_library(hesai_sdk INTERFACE)
    target_link_libraries(hesai_sdk INTERFACE hesai_sdk_lib)
    target_include_directories(hesai_sdk INTERFACE ${HESAI_SDK_DIRECTORY}/driver ${HESAI_SDK_DIRECTORY}/libhesai ${HESAI_SDK_DIRECTORY})
    target_compile_options(hesai_sdk INTERFACE ${HESAI_MACOS_FLAGS})
endif()

message(STATUS "==== Dependencies Configuration Complete ====")

