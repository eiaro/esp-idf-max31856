# SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
#
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)

# Set C++ as the language
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(max31856_cpp)
