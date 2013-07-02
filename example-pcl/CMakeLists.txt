#############################################################################
# KidTsunami - Professional. Live. Video.
#
# File:     CMakeLists.txt
# Author:   Alexander Eichhorn <echa@kidtsunami.com>
# Contents: Template CMake file libopenframeworks project
#
#
# Copyright 2012 KidTsunami. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#############################################################################

# minimum cmake version required
cmake_minimum_required(VERSION 2.8)

# give the project a name
project (example_project)

# Search for libopenframeworks package (will search for OFConfig.cmake,
# of-config.cmake and FindOF.cmake) - we use of-config.cmake which gets
# installed under CMAKE_INSTALL_PREFIX/lib/cmake/of.
#
# You MUST tell cmake where to search for the configuration, either by
#
#    cmake -DCMAKE_PREFIX_PATH=<openframeworks-install-path>/lib/cmake/of
#
# or by setting the environment variable OF_DIR to the above directory.
#
find_package(OF REQUIRED)
include(FindPackageHandleStandardArgs)

# set include directories
include_directories (${OF_INCLUDES} ../SharedCode)

# set cflags
add_definitions (${OF_CFLAGS})

# add link search directories
link_directories (${OF_LIBRARY_DIRS})

# define sources
add_executable(${CMAKE_PROJECT_NAME}
    src/main.cpp
    src/testApp.cpp
)

# link target against libopenframeworks (and implicitly against all depending
# 3rd-party libraries)
target_link_libraries(${CMAKE_PROJECT_NAME} ${OF_LIBRARIES})

# if applicable, set LDFLAGS
if (OF_LDFLAGS)
    set_target_properties(${target_name} PROPERTIES LINK_FLAGS ${OF_LDFLAGS})
endif ()

# soft-link shared data into build directory

# make sure EXTRA_DATA_OUTPUT_DIRECTORY (without the last name part) exists
#get_filename_component(_SHARE_PATH_BASE ${EXTRA_DATA_OUTPUT_DIRECTORY} PATH)

# create base directory
#file(MAKE_DIRECTORY ${_SHARE_PATH_BASE})

# link shared/ from source tree to build tree
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
              ${CMAKE_SOURCE_DIR}/share ${CMAKE_BINARY_DIR}/share)
