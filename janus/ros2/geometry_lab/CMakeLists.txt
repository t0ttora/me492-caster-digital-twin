cmake_minimum_required(VERSION 3.16)
project(janus_geometry_instrumentation LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(janus_geometry_contact SHARED record_contact.cc)
add_library(janus_geometry_mover SHARED move_fixture.cc)
foreach(plugin IN ITEMS janus_geometry_contact janus_geometry_mover)
    target_compile_features(${plugin} PRIVATE cxx_std_17)
    target_link_libraries(${plugin} PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)
    target_compile_options(${plugin} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
