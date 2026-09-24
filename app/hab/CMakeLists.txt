#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(hab LANGUAGES C)

# Suppress strncpy warning from upstream Zephyr USB MSC code (usbd_msc_scsi.c)
# The strncpy usage is correct for fixed-size SCSI fields that don't need null termination
zephyr_compile_options(-Wno-stringop-truncation)

set(CYECCA_PYTHON ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/cyecca_python)

set(CYECCA_PATH $ENV{ZEPHYR_BASE}/../modules/lib/cyecca)

set(flags
  -std=c11
  -Wall
  -Wextra
  -Werror
  -Wstrict-prototypes
  -Waggregate-return
  -Wbad-function-cast
  -Wcast-align
  -Wcast-qual
  -Wfloat-equal
  -Wformat-security
  -Wlogical-op
  -Wmissing-declarations
  # ubxlib includes geographiclib which dir doesn't exist
  #-Wmissing-include-dirs
  -Wmissing-prototypes
  -Wnested-externs
  -Wpointer-arith
  -Wredundant-decls
  -Wsequence-point
  -Wshadow
  -Wstrict-prototypes
  -Wswitch
  -Wundef
  -Wunreachable-code
  -Wunused-but-set-parameter
  -Wwrite-strings
  )
string(JOIN " " flags ${flags})

set(SOURCE_FILES
  src/boot_banner.c
  src/input_mapping.c
  )

if (CONFIG_CEREBRI_HAB_FSM)
  list(APPEND SOURCE_FILES src/fsm.c)
endif()

if (CONFIG_CEREBRI_HAB_ESTIMATE)
  list(APPEND SOURCE_FILES src/estimate.c)
endif()

if (CONFIG_CEREBRI_HAB_ALLOCATION)
  list(APPEND SOURCE_FILES src/allocation.c)
endif()

if (CONFIG_CEREBRI_HAB_LIGHTING)
  list(APPEND SOURCE_FILES src/lighting.c)
endif()

if (CONFIG_CEREBRI_HAB_COMMAND)
  list(APPEND SOURCE_FILES
    src/command.c
    src/mode_bezier.c
    src/mode_velocity.c
    src/mode_attitude.c
    src/mode_attitude_rate.c)
endif()

if (CONFIG_CEREBRI_HAB_POSITION)
  list(APPEND SOURCE_FILES
    src/position.c)
elseif (CONFIG_CEREBRI_HAB_LOGLINEAR_POSITION)
  list(APPEND SOURCE_FILES
    src/log_linear_position.c)
endif()

if (CONFIG_CEREBRI_HAB_ANGULAR_VELOCITY)
  list(APPEND SOURCE_FILES
    src/angular_velocity.c)
endif()

if (CONFIG_CEREBRI_HAB_ATTITUDE)
  list(APPEND SOURCE_FILES
    src/attitude.c)
elseif (CONFIG_CEREBRI_HAB_LOGLINEAR_ATTITUDE)
  list(APPEND SOURCE_FILES
    src/log_linear_attitude.c)
endif()

set(CASADI_DEST_DIR ${CMAKE_BINARY_DIR}/app/hab/casadi)

set(CASADI_FILES
  ${CASADI_DEST_DIR}/hab.c
  ${CASADI_DEST_DIR}/bezier.c
  )

if (CONFIG_CEREBRI_HAB_CASADI)
  list(APPEND SOURCE_FILES ${CASADI_FILES})
endif()

add_custom_command(OUTPUT ${CASADI_DEST_DIR}/hab.c
  COMMAND ${CYECCA_PYTHON} ${CYECCA_PATH}/cyecca/models/hab.py ${CASADI_DEST_DIR}
  DEPENDS ${CYECCA_PATH}/cyecca/models/hab.py)

add_custom_command(OUTPUT ${CASADI_DEST_DIR}/bezier.c
  COMMAND ${CYECCA_PYTHON} ${CYECCA_PATH}/cyecca/models/bezier.py ${CASADI_DEST_DIR}
  DEPENDS ${CYECCA_PATH}/cyecca/models/bezier.py)

set_source_files_properties(
  ${SOURCE_FILES}
  PROPERTIES COMPILE_FLAGS
  "${flags}"
  )

set_source_files_properties(
  ${CASADI_FILES}
  PROPERTIES COMPILE_FLAGS
  "${flags}\
  -Wno-unused-parameter\
  -Wno-missing-prototypes\
  -Wno-missing-declarations\
  -Wno-float-equal")

target_sources(app PRIVATE ${SOURCE_FILES})

target_include_directories(app SYSTEM BEFORE PRIVATE
  ${ZEPHYR_BASE}/include
  ${CMAKE_BINARY_DIR}
  )

if(CONFIG_BOARD STREQUAL "vmu_rt1170")
  zephyr_linker_sources(ITCM_SECTION boards/vmu_rt1170_itcm_linker.ld)
  zephyr_code_relocate(LIBRARY nanopb LOCATION ITCM_TEXT)
endif()

if(CONFIG_BOARD STREQUAL "mr_vmu_tropic")
  zephyr_linker_sources(ITCM_SECTION boards/mr_vmu_tropic_itcm_linker.ld)
  zephyr_code_relocate(LIBRARY nanopb LOCATION ITCM_TEXT)
endif()

if(CONFIG_BOARD STREQUAL "tropic_community")
  # Same coarse relocation vmu_rt1170/mr_navq95a use, without the
  # per-function profiled linker script mr_vmu_tropic also has - that one
  # is generated from a trace of a real running build and references
  # symbols specific to mr_vmu_tropic's own sensor drivers, so it isn't
  # something to copy blind. Revisit once there's a real build to profile.
  zephyr_code_relocate(LIBRARY nanopb LOCATION ITCM_TEXT)
endif()

if(CONFIG_BOARD STREQUAL "mr_navq95a" OR CONFIG_BOARD STREQUAL "mr_navq95b")
  # Relocate code to ITCM
  zephyr_linker_sources(ITCM_SECTION boards/mr_navq95a_itcm_linker.ld)
  zephyr_code_relocate(LIBRARY nanopb LOCATION ITCM_TEXT)
  # Relocate log ring buffer to SDRAM
  zephyr_linker_sources(SECTIONS boards/navq95.ld)
  # Relocate data to OCRAM since zephyr doesn't have an auto spillover
  zephyr_code_relocate(FILES ${SOURCE_FILES} LOCATION OCRAM_DATA)
  zephyr_code_relocate(FILES ${SOURCE_FILES} LOCATION OCRAM_BSS)
  zephyr_code_relocate(FILES ${SOURCE_FILES} LOCATION OCRAM_NOINIT)
  zephyr_code_relocate(LIBRARY cerebri_synapse_topic LOCATION OCRAM_BSS)
  zephyr_code_relocate(LIBRARY cerebri_synapse_topic LOCATION OCRAM_NOINIT)
endif()

# vi: ts=2 sw=2 et
