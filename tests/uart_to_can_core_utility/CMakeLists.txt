cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uart_to_can_core_utility)


set(app_sources
  src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/uart_to_can.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/uart_to_can_core_utility.c
)

target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)