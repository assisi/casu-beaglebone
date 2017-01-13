# Beaglebone 2 pic CMake script

find_path(ZeroMQ_INCLUDE_DIR zmq.hpp)
find_library(ZeroMQ_LIBRARY libzmq.so)
#find_library(Boost_LIBRARIES boost_thread)
#find_library(PROTOBUF_LIBRARY protobuf)

# Needed variables for finding required headers and libraries
SET(Boost_LIBRARIES ${CMAKE_FIND_ROOT_PATH}/usr/lib/arm-linux-gnueabihf/libboost_thread.so ${CMAKE_FIND_ROOT_PATH}/usr/lib/arm-linux-gnueabihf/libboost_system.so
${CMAKE_FIND_ROOT_PATH}/usr/lib/arm-linux-gnueabihf/libboost_date_time.so)
SET(PROTOBUF_LIBRARY ${CMAKE_FIND_ROOT_PATH}/usr/lib/arm-linux-gnueabihf/libprotobuf.so)
SET(PROTOBUF_INCLUDE_DIR ${CMAKE_FIND_ROOT_PATH}/usr/include/google/protobuf)

find_library(yaml-cpp_LIBRARY libyaml-cpp.a)

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
                    ${ZeroMQ_INCLUDE_DIR}
                    ${CMAKE_FIND_ROOT_PATH}/include/)

# Casu firmware sources
set(casu_fw_SOURCES mainApp.cpp
                    CASU_Interface.cpp
                    ../serial/serial.cpp
                    ../i2c/i2cDevice.cpp
                    ../i2c/i2cSlaveMCU.cpp
                    ../ehm/ehm.cpp
                    ${ProtoSources})

# Set -rpath-link linker flags, to prevent weird 2nd-order-dependency linker errors that cost me a handfll of gray hair
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath-link,${CMAKE_FIND_ROOT_PATH}/usr/lib/ -Wl,-rpath,${CMAKE_FIND_ROOT_PATH}/usr/lib/ -Wl,-rpath-link,${CMAKE_FIND_ROOT_PATH}/lib/arm-linux-gnueabihf -Wl,-rpath,${CMAKE_FIND_ROOT_PATH}/lib/arm-linux-gnueabihf")

add_executable(casu-fw ${casu_fw_SOURCES})
target_link_libraries(casu-fw ${ZeroMQ_LIBRARY} ${PROTOBUF_LIBRARY}  ${Boost_LIBRARIES} ${yaml-cpp_LIBRARY} ${CMAKE_FIND_ROOT_PATH}/usr/lib/arm-linux-gnueabihf/libpthread.so)
