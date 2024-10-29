cmake_minimum_required(VERSION 3.29)
project(Frostchef_IoT)

set(CMAKE_CXX_STANDARD 20)

add_executable(Frostchef_IoT main.cpp
        project/mq2-gas-sensor.chip.c)
