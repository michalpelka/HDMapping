cmake_minimum_required(VERSION 4.0.0)

project(proj_test)

add_executable(proj_test proj_test.cpp)

target_link_libraries(proj_test PRIVATE  PROJ::proj)
add_dependencies(proj_test generate_proj_db)