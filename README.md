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

System backup and restore
=========================

Below is the full backup and restore procedure. Depending on your use-case, only some steps may apply.

Backing up to SD card
---------------------

Insert the SD card. Check what's mounted where using the `df` command. Assuming that the internal flash memory (i.e. the device where `/` is mounted) is located at `/dev/mmcblk1` and the SD is at `/dev/mmcblk0`, backing up is performed using the following command:

```
sudo dd if=/dev/mmcblk1 of=/dev/mmcblk0 bs=4M
```

TODO: check if sudo is really necessary
TODO: check additional options for the case where we're backing up to an SD card which is larger than the BeagleBone's internal memory

On Ubuntu 14.04 no progress is displayed. To display the current status of the `dd` process, send the following signal:
```
sudo kill -USR1 $(pgrep ^dd)
```

On ubuntu 16.04 the option `status=progress` will display the progress.

Transferring the image from SD card to the PC
---------------------------------------------

```
sudo dd if=/dev/mmcblk0 | gzip > filename.gz
```

Image backups are available for download [here](http://larics.rasip.fer.hr/laricscloud/index.php/apps/files/?dir=%2Fassisi%2FBeagleBone%20images).

Transferring the image from the PC to the SD card
-------------------------------------------------


Restoring from SD card
----------------------

Insert the SD card into the BeagleBone. Boot from the SD card by holding the "User" button down while powering the BeagleBone (the cape needs to be removed for accessing the "User" button). Restore the image from the SD card to the BeagleBone's internal memory (check which device is which using `df`)

```
sudo dd if=/dev/mmcblk0 of=/dev/mmcblk` bs=4M
```

If the SD card is larger than the BeagleBone's internal memory, you might get a "Not enough space on the device" error which can be safely ignored.

