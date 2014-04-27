# Beaglebone 2 pic

# External dependencies
#set(ZeroMQ_INCLUDE_DIR /usr/local/include)
#set(ZeroMQ_LIBRARY /usr/local/lib/libzmq.so)

#set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
find_library(ZeroMQ_LIBRARY zmq)
#set(ZeroMQ_LIBRARY libzmq.so)
find_library(Boost_LIBRARIES boost_thread)

# Second order dependencies, should be handled by -rpath-link
# http://stackoverflow.com/questions/21113434/error-cross-compiling-cpp-netlib-for-raspbian
# or even better:
# http://sysprogs.com/blog/?p=54
find_library(PGM_LIBRARY pgm)
find_library(Z_LIBRARY z)
find_library(UUID_LIBRARY uuid)
message("${Z_LIBRARY}")

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

#set(CMAKE_SKIP_BUILD_RPATH FALSE)
#set(CMAAKE_BUILD_WITH_INSTALL_RPATH FALSE)
#set(CMAKE_INSTALL_RPATH $ENV{HOME}/dev/bbone/lib/arm-linux-gnueabihf
#                        $ENV{HOME}/dev/bbone/usr/lib)
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath-link,/home/damjan/dev/bbone/lib/arm-linux-gnueabihf -Wl,-rpath-link,/home/damjan/dev/bbone/usr/lib")

add_executable(casu-fw ${casu_fw_SOURCES})
target_link_libraries(casu-fw ${ZeroMQ_LIBRARY} ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES})
