#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# Explicitly set overlay file path
set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/samd21_xpro.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE src/main.c)

# Only compile ethernet example if networking is enabled
if(CONFIG_NETWORKING)
    target_sources(app PRIVATE src/ethernet_example.c)
endif()