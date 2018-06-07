cmake_minimum_required (VERSION 2.8)
project (hello_pixy CXX)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake" )

LINK_DIRECTORIES(/usr/lib/libftdi1)
# Add sources here... #
add_executable (hello_pixy hello_pixy.cpp gpio.cpp robot.cpp)


# libpixyusb should always come before libboost and libusb #
target_link_libraries (hello_pixy pixyusb)

find_package ( libpixyusb REQUIRED )
find_package ( libusb-1.0 REQUIRED )
find_package ( Boost 1.49 COMPONENTS thread system chrono REQUIRED)
target_compile_options(hello_pixy PRIVATE -fpermissive)
target_link_libraries (hello_pixy ${Boost_LIBRARIES})
target_link_libraries (hello_pixy ${LIBUSB_1_LIBRARY})
TARGET_LINK_LIBRARIES(hello_pixy /usr/lib/x86_64-linux-gnu/libftdi1.a)

file(STRINGS "cmake/VERSION" LIBPIXY_VERSION)
add_definitions(-D__LIBPIXY_VERSION__="${LIBPIXY_VERSION}")

include_directories (/home/jan/pixy2/pixy/src/host/hello_pixy
                     src
                     include
                     ../../common
                     ${Boost_INCLUDE_DIR}
                     ${LIBUSB_1_INCLUDE_DIRS})

