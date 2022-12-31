#-------------------------------------------------------------------------------
# ENS160+ATH2x Board Sample
#
# Copyright (c) 2022 Russ Webber
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ens160_ath2x_board LANGUAGES C)

target_sources(app PRIVATE src/main.c)
