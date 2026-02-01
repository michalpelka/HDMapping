cmake_minimum_required(VERSION 4.0.0)

# Download SQLite precompiled zip
set(SQLITE_TOOLS_DOWNLOAD_ZIP_URL "https://www.sqlite.org/2026/sqlite-tools-win-x64-3510200.zip")
set(SQLITE_TOOLS_DOWNLOAD_ZIP_SHA256 "042805d77076e2b806c86b1ac4082d65c2f2d4bdef5ce8995eed7845878fd69f")
set(SQLITE_TOOLS_DOWNLOAD_ZIP "${CMAKE_BINARY_DIR}/sqlite-tools-win-x64-3510200.zip")


if(NOT EXISTS ${SQLITE_TOOLS_DOWNLOAD_ZIP})
    message(STATUS "Downloading SQLITE Tools...")
    file(DOWNLOAD ${SQLITE_TOOLS_DOWNLOAD_ZIP_URL} ${SQLITE_TOOLS_DOWNLOAD_ZIP} SHOW_PROGRESS)
    # Verify SHA256 checksum
    file(SHA256 ${SQLITE_TOOLS_DOWNLOAD_ZIP} SQLITE_TOOLS_DOWNLOAD_ZIP_SHA256_COMPUTED)
    if(NOT ${SQLITE_TOOLS_DOWNLOAD_ZIP_SHA256_COMPUTED} STREQUAL ${SQLITE_TOOLS_DOWNLOAD_ZIP_SHA256})
        message(FATAL_ERROR "SHA256 checksum mismatch for downloaded SQLITE_TOOLS_DOWNLOAD_ZIP_URL.")
    endif()
endif()

# Extract SQLITE_ZIP
set(SQLITE_EXTRACT_DIR "${CMAKE_BINARY_DIR}/3rd_binary/SQLITETools")
if(NOT EXISTS ${SQLITE_EXTRACT_DIR})
    message(STATUS "Extracting SQLITE Tools...")
    file(ARCHIVE_EXTRACT
            INPUT ${SQLITE_TOOLS_DOWNLOAD_ZIP}
            DESTINATION ${SQLITE_EXTRACT_DIR}
    )
endif()
if(EXISTS ${SQLITE_EXTRACT_DIR})
    set (SQLITE_DIR ${CMAKE_BINARY_DIR}/3rd_binary/SQLITETools/)
    set (SQLite3_EXECUTABLE ${CMAKE_BINARY_DIR}/3rd_binary/SQLITETools/sqlite3.exe)
endif()

if(NOT SQLite3_EXECUTABLE)
    message(FATAL_ERROR "sqlite3.exe not found after extraction in ${SQLITE_EXTRACT_DIR}")
endif()

# expose to proj CMake
set(EXE_SQLITE3 "${SQLite3_EXECUTABLE}" CACHE FILEPATH "Path to sqlite3.exe" FORCE)
message(STATUS "Using sqlite3.exe: ${EXE_SQLITE3}")