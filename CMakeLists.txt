project(trak_driver)
cmake_minimum_required(VERSION 3.16)
set(TARGET_NAME trak_driver)

add_library(${TARGET_NAME}.x64 SHARED
  driverlog.cpp
  driverlog.h
  driver_sample.cpp
)

add_library(${TARGET_NAME}.x86 SHARED
  driverlog.cpp
  driverlog.h
  driver_sample.cpp
)

set_target_properties(${TARGET_NAME}.x86 PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")

add_definitions(-DDRIVER_SAMPLE_EXPORTS)

find_library(openvr REQUIRED)
find_library(serial REQUIRED)
if(WIN32)
  find_library(enumser REQUIRED)
endif()

target_link_libraries(${TARGET_NAME}.x86
  ${OPENVR_LIBRARIES}
  ${SERIAL_LIBRARIES}
  ${ENUMSER_LIBRARIES}
  ${CMAKE_DL_LIBS}
)
target_link_libraries(${TARGET_NAME}.x64
  ${OPENVR_LIBRARIES}
  ${SERIAL_LIBRARIES}
  ${ENUMSER_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

