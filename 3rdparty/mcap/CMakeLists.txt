include(FetchContent)
# Tarball of the tag rather than a git clone: the foxglove/mcap monorepo's history
# is ~140 MB, the snapshot under 1 MB (only the header-only C++ library is used).
FetchContent_Declare(mcap
        URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v1.4.0.tar.gz
        URL_HASH SHA256=64ff3e51119f37ffcfaf9deecbd987a7cb4d4d9035d74a3fd3773395a470fda1
)
FetchContent_MakeAvailable(mcap)

add_library(mcap INTERFACE)
target_include_directories(mcap INTERFACE ${mcap_SOURCE_DIR}/cpp/mcap/include)

# mcap's reader/writer .inl always compile in LZ4/ZSTD (de)compression paths,
# even for callers that only ever write Compression::None -- so the symbols
# must be linked unconditionally, not just when compression is actually used.
#
# Linux and macOS link the system liblz4/libzstd; Windows links the static
# lz4_static/zstd_static built from the lz4 and zstd submodules (see
# cmake/dependencies.cmake).
if(WIN32)
    target_link_libraries(mcap INTERFACE lz4_static zstd_static)
else()
    find_library(MCAP_LZ4_LIBRARY NAMES lz4)
    find_library(MCAP_ZSTD_LIBRARY NAMES zstd)
    find_path(MCAP_LZ4_INCLUDE_DIR NAMES lz4frame.h)
    find_path(MCAP_ZSTD_INCLUDE_DIR NAMES zstd.h)
    if(NOT MCAP_LZ4_LIBRARY OR NOT MCAP_ZSTD_LIBRARY OR NOT MCAP_LZ4_INCLUDE_DIR OR NOT MCAP_ZSTD_INCLUDE_DIR)
        message(FATAL_ERROR "mcap requires liblz4 and libzstd (dev packages) to build against")
    endif()
    target_link_libraries(mcap INTERFACE ${MCAP_LZ4_LIBRARY} ${MCAP_ZSTD_LIBRARY})
    target_include_directories(mcap INTERFACE ${MCAP_LZ4_INCLUDE_DIR} ${MCAP_ZSTD_INCLUDE_DIR})
endif()
