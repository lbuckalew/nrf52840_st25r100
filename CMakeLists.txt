# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Add optix_zephyr custom out-of-tree zephyr resources
set(EXTERNAL_Z_MODULE ${CMAKE_CURRENT_SOURCE_DIR}/module)
list(APPEND ZEPHYR_EXTRA_MODULES ${EXTERNAL_Z_MODULE})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(st25r100_testing)

target_sources(app PRIVATE src/main.c)
