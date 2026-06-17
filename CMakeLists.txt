# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sub_controller)

set(SUB_CONTROLLER_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uart_to_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uart_to_can_core_utility.c
)
target_sources(app PRIVATE ${SUB_CONTROLLER_SRC})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(app PRIVATE -Werror -Wall -Wextra -Wshadow -Wdouble-promotion -Wformat)
target_link_libraries(app PRIVATE)



