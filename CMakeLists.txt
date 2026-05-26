# Copyright (C) 2025 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.10)
project(manipulator LANGUAGES C CXX)

set(CMAKE_C_STANDARD   99)
set(CMAKE_C_STANDARD_REQUIRED   ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MANIP_LOCAL_MOTOR_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../../peripherals/motor)

find_package(Threads REQUIRED)

# ===========================================================================
# Source files
# ===========================================================================

set(MANIP_SOURCES
    src/manipulator.c
    src/drivers/drv_dummy.c
    src/kinematics/kinematics.c
    src/kinematics/kinematics_dummy.c
)

# --- SO-101 driver (requires motor headers) --------------------------------
option(MANIP_ENABLE_SO101_DRIVER "Enable SO-101 UART driver" ON)

if(MANIP_ENABLE_SO101_DRIVER)
    find_path(MOTOR_INCLUDE_DIR motor.h
        HINTS
            ${MANIP_LOCAL_MOTOR_DIR}/include
            ${MOTOR_INCLUDE_PATH})

    if(MOTOR_INCLUDE_DIR)
        message(STATUS "[manipulator] SO-101 driver enabled (motor: ${MOTOR_INCLUDE_DIR})")
        list(APPEND MANIP_SOURCES src/drivers/drv_uart_so101.c)
        set(HAVE_SO101_DRIVER TRUE)
    else()
        message(STATUS "[manipulator] motor headers not found — SO-101 driver disabled")
        set(HAVE_SO101_DRIVER FALSE)
    endif()
else()
    set(HAVE_SO101_DRIVER FALSE)
endif()

# For shared-library consumers (e.g. Python ctypes), motor symbols must be
# resolved while building manipulator itself rather than only when linking
# test executables. Ensure the motor target exists before manipulator links.
if(HAVE_SO101_DRIVER AND NOT TARGET motor)
    set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech"
        CACHE STRING "Enabled motor drivers (semicolon-separated). Empty means auto-detect." FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "Build test programs" FORCE)
    add_subdirectory(
        ${MANIP_LOCAL_MOTOR_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/motor)
endif()

# --- Pinocchio kinematics (optional) --------------------------------------
option(MANIP_ENABLE_PINOCCHIO "Enable Pinocchio kinematics solver" ON)

if(MANIP_ENABLE_PINOCCHIO)
    find_package(pinocchio QUIET)
    if(pinocchio_FOUND)
        message(STATUS "[manipulator] Pinocchio found — kinematics enabled")
        list(APPEND MANIP_SOURCES src/kinematics/kinematics_pinocchio.cpp)
        set(HAVE_PINOCCHIO TRUE)
    else()
        message(STATUS "[manipulator] Pinocchio not found (see README for source build instructions)")
        set(HAVE_PINOCCHIO FALSE)
    endif()
endif()

# ===========================================================================
# Library target
# ===========================================================================

add_library(manipulator SHARED ${MANIP_SOURCES})

set_target_properties(manipulator PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    OUTPUT_NAME manipulator)

target_include_directories(manipulator
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(manipulator PRIVATE Threads::Threads m)

if(HAVE_SO101_DRIVER)
    target_compile_definitions(manipulator PRIVATE HAVE_MOTOR HAVE_SO101_DRIVER)
    target_include_directories(manipulator PRIVATE ${MOTOR_INCLUDE_DIR})
    if(TARGET motor)
        target_link_libraries(manipulator PUBLIC motor)
        target_link_options(manipulator PRIVATE
            "LINKER:--whole-archive,$<TARGET_FILE:motor>,--no-whole-archive")
    endif()
endif()

if(HAVE_PINOCCHIO)
    target_compile_definitions(manipulator PRIVATE HAVE_PINOCCHIO)
    target_link_libraries(manipulator PRIVATE pinocchio::pinocchio)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(manipulator PRIVATE -Wall -Wextra)
endif()

# Copy URDF files so default paths work at runtime
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/urdf
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# ===========================================================================
# Install
# ===========================================================================

install(TARGETS manipulator
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)

install(FILES
    include/manipulator.h
    include/kinematics_interface.h
    DESTINATION include)

install(DIRECTORY urdf/
    DESTINATION share/manipulator/urdf
    FILES_MATCHING PATTERN "*.urdf")

# ===========================================================================
# Tests
# ===========================================================================

option(MANIP_BUILD_TESTS   "Build unit tests"                      ON)
option(MANIP_BUILD_HW_TEST "Build SO-101 hardware test program"    OFF)
option(MANIP_BUILD_BENCHMARKS "Build manipulator benchmark programs" ON)

if(MANIP_BUILD_TESTS)
    enable_testing()

    # Ensure motor target exists if SO-101 driver is enabled
    if(HAVE_SO101_DRIVER AND NOT TARGET motor)
        set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech"
            CACHE STRING "Enabled motor drivers (semicolon-separated). Empty means auto-detect." FORCE)
        set(BUILD_TESTS OFF CACHE BOOL "Build test programs" FORCE)
        add_subdirectory(
            ${MANIP_LOCAL_MOTOR_DIR}
            ${CMAKE_CURRENT_BINARY_DIR}/motor)
    endif()

    # Framework test (uses dummy driver from drv_dummy.c)
    # Need --whole-archive to preserve constructor for driver auto-registration
    add_executable(test_manipulator test/test_manipulator.c)
    # manipulator already carries its dependent libraries; avoid re-whole-archiving
    # transitive dependencies here to prevent duplicate static objects.
    if(HAVE_SO101_DRIVER AND TARGET motor)
        target_link_libraries(test_manipulator PRIVATE
            manipulator
            Threads::Threads)
    else()
        target_link_libraries(test_manipulator PRIVATE
            manipulator
            Threads::Threads)
    endif()
    set_target_properties(test_manipulator PROPERTIES LINKER_LANGUAGE CXX)
    add_test(NAME manipulator_test COMMAND test_manipulator)

    # Kinematics test (standalone, but needs motor if SO-101 driver is enabled)
    if(HAVE_PINOCCHIO)
        # Ensure motor target exists if SO-101 driver is enabled
        if(HAVE_SO101_DRIVER AND NOT TARGET motor)
            set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech"
                CACHE STRING "Enabled motor drivers (semicolon-separated). Empty means auto-detect." FORCE)
            set(BUILD_TESTS OFF CACHE BOOL "Build test programs" FORCE)
            add_subdirectory(
                ${MANIP_LOCAL_MOTOR_DIR}
                ${CMAKE_CURRENT_BINARY_DIR}/motor)
        endif()

        add_executable(test_kinematics test/test_kinematics.c)
        # manipulator already carries its dependent libraries; avoid re-whole-archiving
        # transitive dependencies here to prevent duplicate static objects.
        if(HAVE_SO101_DRIVER AND TARGET motor)
            target_link_libraries(test_kinematics PRIVATE
            manipulator
                Threads::Threads m)
        else()
            target_link_libraries(test_kinematics PRIVATE
            manipulator
                Threads::Threads m)
        endif()
        set_target_properties(test_kinematics PROPERTIES LINKER_LANGUAGE CXX)
        add_test(NAME kinematics_test COMMAND test_kinematics)
    endif()

    add_executable(test_dummy_kinematics_api test/test_dummy_kinematics_api.c)
    target_link_libraries(test_dummy_kinematics_api PRIVATE
        manipulator
        Threads::Threads m)
    set_target_properties(test_dummy_kinematics_api PROPERTIES LINKER_LANGUAGE CXX)
endif()

if(MANIP_BUILD_BENCHMARKS)
    add_executable(benchmark_dummy_manipulator test/benchmark_dummy_manipulator.c)
    target_link_libraries(benchmark_dummy_manipulator PRIVATE
        manipulator
        Threads::Threads m)
    set_target_properties(benchmark_dummy_manipulator PROPERTIES LINKER_LANGUAGE CXX)
endif()

if(MANIP_BUILD_HW_TEST AND HAVE_SO101_DRIVER)
    find_path(GRASP_INCLUDE_DIR grasp.h
        HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../grasp/include)

    if(MOTOR_INCLUDE_DIR AND GRASP_INCLUDE_DIR)
        if(NOT TARGET motor)
            set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech"
                CACHE STRING "Enabled motor drivers (semicolon-separated). Empty means auto-detect." FORCE)
            set(BUILD_TESTS OFF CACHE BOOL "Build test programs" FORCE)
            add_subdirectory(
                ${MANIP_LOCAL_MOTOR_DIR}
                ${CMAKE_CURRENT_BINARY_DIR}/motor)
        endif()
        if(NOT TARGET grasp)
            add_subdirectory(
                ${CMAKE_CURRENT_SOURCE_DIR}/../grasp
                ${CMAKE_CURRENT_BINARY_DIR}/grasp)
        endif()

        add_executable(test_hw_so101 test/test_hw_so101.c)
        # manipulator and grasp are shared libraries; linking them normally avoids
        # repeatedly pulling in transitive static archives such as motor/libgcc.
        # gpiod: needed by drv_pwm_generic compiled into the motor static lib
        target_link_libraries(test_hw_so101 PRIVATE
            manipulator grasp
            Threads::Threads m gpiod)
        target_include_directories(test_hw_so101 PRIVATE
            ${GRASP_INCLUDE_DIR} ${MOTOR_INCLUDE_DIR})
        set_target_properties(test_hw_so101 PROPERTIES LINKER_LANGUAGE CXX)

        message(STATUS "[manipulator] HW test enabled: test_hw_so101")
    else()
        message(WARNING "[manipulator] HW test needs motor + grasp — skipping")
    endif()
endif()
