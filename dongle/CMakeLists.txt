cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(charybdis_dongle)

target_sources(app PRIVATE src/main.c)

if(DEFINED DONGLE_TARGET_NAME)
    target_compile_definitions(app PRIVATE DONGLE_TARGET_NAME="${DONGLE_TARGET_NAME}")
endif()
