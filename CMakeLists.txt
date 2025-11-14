cmake_minimum_required(VERSION 3.10)
project(usb_serial_transport)

find_package(ament_cmake REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)
find_package(cobs_c REQUIRED)
find_package(Threads REQUIRED)
find_package(tl_expected REQUIRED)

set(_usb_serial_export_deps cobs_c Threads tl_expected)

set(_usb_serial_asio_include_dirs "")
if(TARGET asio::asio)
  get_target_property(_asio_includes asio::asio INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT _asio_includes OR _asio_includes STREQUAL "_asio_includes-NOTFOUND")
    set(_asio_includes ${ASIO_INCLUDE_DIRS})
  endif()
  set(_usb_serial_asio_include_dirs ${_asio_includes})
elseif(TARGET ASIO::asio)
  get_target_property(_asio_includes ASIO::asio INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT _asio_includes OR _asio_includes STREQUAL "_asio_includes-NOTFOUND")
    set(_asio_includes ${ASIO_INCLUDE_DIRS})
  endif()
  set(_usb_serial_asio_include_dirs ${_asio_includes})
elseif(ASIO_INCLUDE_DIRS)
  set(_usb_serial_asio_include_dirs ${ASIO_INCLUDE_DIRS})
else()
  message(FATAL_ERROR
    "The 'ASIO' package did not provide include directories. Check your ROS ASIO installation.")
endif()

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
if(NOT _usb_serial_asio_include_dirs)
  message(FATAL_ERROR "ASIO include directories were not provided through targets or variables.")
endif()

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${_usb_serial_asio_include_dirs})

target_link_libraries(${PROJECT_NAME} INTERFACE
  cobs_c::cobs_c
  Threads::Threads
  tl_expected::tl_expected)

target_compile_definitions(${PROJECT_NAME} INTERFACE ASIO_STANDALONE)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${_usb_serial_export_deps})

ament_package()
