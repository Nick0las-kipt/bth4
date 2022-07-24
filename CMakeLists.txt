# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
# Re-direct the directory where the 'boards' directory is found from
# $ZEPHYR_BASE to this directory.
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(BOARD nrf52_smaq2)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(sma_q2_bringup)

target_sources(app PRIVATE 
	src/main.cpp
	src/inits.c
	src/charger.c
	src/lpm013m126.c
	src/backlight.c
	src/slowsensors.c
	src/fonts.c
	src/buttons.c
	src/msm/strservice.c
	src/bt.c
	src/rtc.c
	src/watchdog.c
	src/msm/msm_lpm13_drawer.cpp
	src/menu.cpp
	src/dispservice.cpp
	src/graph.c
	src/sensorlog.c
	src/calculs/meteomath.c
	src/calculs/daynight.c
	src/calculs/daynightimpl.c
	)

zephyr_library_include_directories(src/)
zephyr_library_include_directories(config/)