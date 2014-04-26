# Beaglebone 2 pic

# External dependencies
find_package(ZeroMQ REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Protobuf REQUIRED)

# Set up compilation of protobuffer files
set(Proto_FILES ../msg/base_msgs.proto
                ../msg/dev_msgs.proto
                ../msg/sim_msgs.proto)

set(PROTOBUF_IMPORT_DIRS ${PROTOBUF_IMPORT_DIRS}
                         ${CMAKE_SOURCE_DIR}/msg)
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${Proto_FILES})

# Set up include directories
include_directories(${casu_beaglebone_SOURCE_DIR/serial}
                    ${casu_beaglebone_SOURCE_DIR/i2c}
                    ${casu_beaglebone_SOURCE_DIR/ehm}
                    ${ZeroMQ_INCLUDE_DIR})

# Casu firmware sources
set(casu_fw_SOURCES testApp.cpp
                    BBBInterface.cpp
                    serial.cpp
                    I2CDevice.cpp
                    I2CSlavePIC.cpp
                    ehm.cpp
                    ${ProtoSources})

add_executable(casu-fw ${casu_fw_SOURCES})
target_link_libraries(casu-fw ${ZeroMQ_LIBRARY} ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES})
