# Beaglebone 2 pic CMake script

#set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
find_library(ZeroMQ_LIBRARY zmq)
find_library(Boost_LIBRARIES boost_thread)

#find_package(Boost COMPONENTS thread REQUIRED)
find_package(Protobuf REQUIRED)

# Set up compilation of protobuffer files
set(Proto_FILES ../msg/base_msgs.proto
                ../msg/dev_msgs.proto
                ../msg/sim_msgs.proto)

set(PROTOBUF_IMPORT_DIRS ${PROTOBUF_IMPORT_DIRS}
                         ${CMAKE_SOURCE_DIR}/msg)
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${Proto_FILES})

# Set up include directories
include_directories(${CMAKE_SOURCE_DIR}/serial
                    ${CMAKE_SOURCE_DIR}/i2c
                    ${CMAKE_SOURCE_DIR}/ehm
                    ${CMAKE_BINARY_DIR}/beaglebone2pic # This is for protobuffer stuff!
                    )

# Casu firmware sources
set(casu_fw_SOURCES TestApp.cpp
                    BBBInterface.cpp
                    ../serial/serial.cpp
                    ../i2c/I2CDevice.cpp
                    ../i2c/I2CSlavePIC.cpp
                    ../ehm/ehm.cpp
                    ${ProtoSources})

set(PROTOBUF_LIBRARY libprotobuf.so)

# Set -rpath-link linker flags, to prevent weird 2nd-order-dependency linker errors that cost me a handfll of gray hair
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath-link,$ENV{CROSS_ENV_ROOT}/lib/arm-linux-gnueabihf -Wl,-rpath-link,$ENV{CROSS_ENV_ROOT}/usr/lib")

add_executable(casu-fw ${casu_fw_SOURCES})
target_link_libraries(casu-fw ${ZeroMQ_LIBRARY} ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES})
