include(FetchContent)

FetchContent_Declare(
    sqlite_amalgamation
    URL https://www.sqlite.org/2023/sqlite-amalgamation-3430000.zip    # Use correct version and link
)
FetchContent_MakeAvailable(sqlite_amalgamation)

add_library(sqlite3 STATIC
  ${sqlite_amalgamation_SOURCE_DIR}/sqlite3.c
)

target_include_directories(sqlite3 PUBLIC
  $<BUILD_INTERFACE:${sqlite_amalgamation_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

if (MSVC)
  target_compile_options(sqlite3 PRIVATE /bigobj /Zm300)
endif()

add_library(SQLite::SQLite3 ALIAS sqlite3)

install(TARGETS sqlite3
  EXPORT targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(SQLITE3_INCLUDE_DIR
    "${sqlite_amalgamation_SOURCE_DIR}"
    CACHE PATH "SQLite3 include dir"
    FORCE
)

set(SQLITE3_LIBRARY
    "$<TARGET_FILE:sqlite3>"
    CACHE FILEPATH "SQLite3 library file"
    FORCE
)

set(SQLITE3_FOUND TRUE CACHE BOOL "" FORCE)
set(SQLITE3_VERSION "3.43.0" CACHE STRING "" FORCE)
MESSAGE(STATUS "SQLITE3_INCLUDE_DIR is ${SQLITE3_INCLUDE_DIR}")
MESSAGE(STATUS "SQLITE3_LIBRARY is ${SQLITE3_LIBRARY}")
install(FILES
  "${sqlite_amalgamation_SOURCE_DIR}/sqlite3.h"
  DESTINATION include
)
message(STATUS,"SQLITE3_FOUND: ${SQLITE3_FOUND}")



