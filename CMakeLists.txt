cmake_minimum_required(VERSION 3.1)

project(intel_tera_mavlink)

find_package(serial REQUIRED)
set(MAVLINK_DIALECT common)


add_executable(intel_tera_mavlink src/teraranger.cpp src/helper_lib.cpp src/mavlink_tcp.cpp src/main.cpp)

set_property(TARGET intel_tera_mavlink PROPERTY CXX_STANDARD 11)
set_property(TARGET intel_tera_mavlink PROPERTY COMPILE_FLAGS "-Wall -Wextra -Wshadow -Wundef -Wformat=2 -Wlogical-op -Wsign-compare -Wformat-security -Wformat-nonliteral -Wpointer-arith -Winit-self -Wmissing-noreturn -Wstrict-aliasing=2 -Wno-overlength-strings -Wno-missing-field-initializers -Wuninitialized")

target_include_directories(intel_tera_mavlink PUBLIC ${CMAKE_SOURCE_DIR}/modules/mavlink_c/${MAVLINK_DIALECT})
target_link_libraries(intel_tera_mavlink PUBLIC ${CMAKE_SOURCE_DIR}/modules/dserial)

target_link_libraries(intel_tera_mavlink OpticalFlow)
target_link_libraries(intel_tera_mavlink pthread)
install(TARGETS intel_tera_mavlink DESTINATION bin)

configure_file(aero-teraranger.service.in aero-teraranger.service)
