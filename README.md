Abstract
========

Source code for the "Casu firmware" running on the Beaglebone Black.

Building the software
=====================

Point the environment variable `CROSS_ENV_ROOT` to the root of your crosscompiling environment (i.e., the place where you have copied the relevant parts of the Beaglebone filesystem :)

Use the standard cmake build procedure, using the provided toolchain file. From the top-level directory:

    $ mkdir build-arm
    $ cd build-arm
    $ cmake .. -DCMAKE_TOOLCHAIN_FILE=Toolchain-arm-linux-gnueabihf.cmake
    $ make


