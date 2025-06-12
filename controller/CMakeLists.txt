# CMakeLists.txt
#
# Copyright (c) 2022 Daniel Collins
#
# This file is part of controller.
#
# controller is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License, version 3 as published by the
# Free Software Foundation.
#
# controller is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.
#
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# controller. If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


# Set standard CMake build type options
## Default C compiler flags
set(CMAKE_C_FLAGS_DEBUG_INIT "-g3 -Og -Wall -Wextra -DDEBUG")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG_INIT}" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELEASE_INIT "-O3 -Wall")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE_INIT}" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_MINSIZEREL_INIT "-Os -Wall")
set(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL_INIT}" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g -Wall")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_ASM_FLAGS_RELWITHDEBINFO_INIT}" CACHE STRING "" FORCE)
## Default C++ compiler flags
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-g3 -Og -Wall -Wextra -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG_INIT}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-O3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE_INIT}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_MINSIZEREL_INIT "-Os -Wall")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL_INIT}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT "-O2 -g -Wall")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_ASM_FLAGS_RELWITHDEBINFO_INIT}" CACHE STRING "" FORCE)



# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
project(controller C CXX ASM)
pico_sdk_init()

add_executable(controller)
# pico_set_float_implementation(controller PICO) 
pico_generate_pio_header(controller ${CMAKE_CURRENT_LIST_DIR}/lib/ws2812_driver.pio)
pico_generate_pio_header(controller ${CMAKE_CURRENT_LIST_DIR}/lib/rotary.pio)

target_include_directories(controller PRIVATE
       "./lib"
       "./pages"
   )

target_sources(controller PRIVATE
			  controller.c	
           		  "./lib/ssd1306_i2c_driver.c"
			  "./lib/moving_average.c"
			  "./lib/ws2812_driver.c"
			  "./lib/rotary_encoder.c"
			  "./lib/ads1115.c"
			  "./lib/bits8.c"
			  "./lib/ui.c"
			  "./lib/common.c"
			  "./pages/channel.c")

pico_set_program_name(controller "controller")
pico_set_program_version(controller "1.0.1")

# no_flash means the target is to run from RAM
# pico_set_binary_type(controller no_flash)

pico_enable_stdio_uart(controller 1)
pico_enable_stdio_usb(controller 0)

# Add the standard library to the build
target_link_libraries(controller pico_stdlib)

# Add any user requested libraries
target_link_libraries(controller
        hardware_i2c
	hardware_adc
        hardware_dma
        hardware_pio
        hardware_clocks
	hardware_flash
	pico_multicore
	pico_float
	pico_stdlib)

pico_add_extra_outputs(controller)
