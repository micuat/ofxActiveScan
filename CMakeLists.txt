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
#     http://www.apache.cpprg/licenses/LICENSE-2.0
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

# find_package(OF REQUIRED)

# set include directories
set (OF_INCLUDES
    ${OF_INCLUDES}
    /home/sre/nhieda/intel/mkl/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ProCamTools
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ProCamTools/common
    ${CMAKE_CURRENT_SOURCE_DIR}/src/levmar
    PARENT_SCOPE
)

# set cflags
#set (OF_CFLAGS
#    ${OF_CFLAGS}
#    PARENT_SCOPE
#)

# add link search directories
set (OF_LIBRARY_DIRS
    ${OF_LIBRARY_DIRS}
    /home/sre/nhieda/intel/mkl/lib/ia32
    PARENT_SCOPE
)

# define sources
set (OFX_PROCAMTOOLS_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ProCamTools/FundamentalMatrix.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/levmar/Axb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/levmar/lmbc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/levmar/lmblec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/levmar/lmbleic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/levmar/lmlec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/levmar/lm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/levmar/misc.c
    PARENT_SCOPE
)

# if applicable, set LDFLAGS
set (OF_LIBRARIES
#set (OF_LDFLAGS
    ${OF_LIBRARIES}
#    ${OF_LDFLAGS}
    -lmkl_core
    -lmkl_pgi_thread
    -lmkl_rt
    -lmkl_scalapack_core
    -lmkl_sequential
    PARENT_SCOPE
)
