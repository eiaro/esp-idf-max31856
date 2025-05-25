# SPDX-FileCopyrightText: 2025 Ronny Eia <3652665+eiaro@users.noreply.github.com>
#
# SPDX-License-Identifier: MIT

idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES max31856)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)
