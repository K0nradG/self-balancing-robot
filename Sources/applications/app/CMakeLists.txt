# Copyright 2026 Filip Dymczyk and Konrad Grucel

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(self-balancing-robot C CXX)

FILE(GLOB app_sources src/*.cpp)

target_sources(app PRIVATE ${app_sources})
