
# Beaglebone Environment Setup Instructions

These instructions are intended for installation and setup of a Beaglebone Green, from a workstation running Ubuntu 16.04.

### Contents

  1. [Ubuntu 16.04 Installation](#ubuntu-1604-installation)
    1. [Image Download and SD Card Preparation](#image-download-and-sd-card-preparation)
    2. [Flashing the Image to Beaglebone](#flashing-the-image-to-beaglebone)
  3. [Resizing the Image Partition](#resizing-the-image-partition)
  4. [Network Setup](#network-setup)
  5. [Firmware Installation](#firmware-installation)
    1. [Firmware and Hardware Dependencies](#firmware-and-hardware-dependencies)
    2. [Installation](#installation)
    3. [Setting up NTP Update and Firmware Launch on Startup](#setting-up-ntp-update-and-firmware-launch-on-startup)
    4. [Setting up CASU Firmware Status Display on Startup](#setting-up-casu-firmware-status-display-on-startup)
  6. [Locale Warnings](#locale-warnings)
  7. [Building the Linux Kernel](#building-the-linux-kernel)
  8. [Enabling the RTC Clock](#enabling-the-rtc-clock)

## Ubuntu 16.04 Installation
If you already have prepared a microSD card, skip to [Flashing the Image to Beaglebone](#flashing-the-image-to-beaglebone).

### Image Download and SD Card Preparation

[Get a prebuilt image](http://elinux.org/BeagleBoardUbuntu#Demo_Image) for Beaglebone/Beaglebone Black. _Do not_ download a flasher image. Follow the steps described on the link:

- download the .tar file
- verify and unpack the image
- locate your SD card (`fdisk -l` is useful)
- flash the image to the SD card using `dd`. The device file of the SD card reader can vary between different manufacturers.  `if` is the source, and `of` is the destination. Be careful not to mix the two.

 Warning: this process will wipe all data on your SD card.

      sudo dd if=/path/to/image of=/dev/sd_card_device bs=4M status=progress

### Flashing the Image to Beaglebone

1. Insert the microSD card into the Beaglebone microSD slot
2. While pressing the USER button next to the slot, plug the board to your station via usb (micro USB cable). Continue pressing the button while four blue LEDs light up and start blinking "randomly"
3. You should have a new interface (a virtual network device) available, called _Seeed BeagleBoneGreen_. In the network manager, create a new connection using this interface, and set a IP address of your machine to `192.168.7.1`, with the subnet mask `255.255.255.0`.
4. Connect to the board via SSH from a terminal under the default user _ubuntu_. The default board IP is 192.168.7.2. The default password for the image is _temppwd_.

        ssh ubuntu@192.168.7.2

4. Determine the path to the SD card and local flash memory using `fdisk -l` and `df -h`. Comparing the memory sizes can be helpful. Usually, the card is `mmcblk0`, and the local flash memory `mmcblk1`.
6. Copy the image from SD to local flash with `dd`. __Replace paths if they differ in your case.__ As before, be careful not to confuse the `if` and `of` devices.

        sudo dd if=/dev/mmcblk0 of=/dev/mmcblk1 bs=4M status=progress

 If your card is larger than the Beaglebone flash memory, an error will be shown after 3.6Gb of data is transferred. It is safe to ignore this error.

7. Reboot the board without SD card (unplug from power, unplug card, plug to power)

## Resizing the Image Partition

If an image of less than 4GB available is flashed to the BBG, the full space on the flash memory will not be utilized. This can be fixed by resizing the partition, as follows:

    sudo fdisk /dev/mmcblk1

The resizing is carried out by deleting the existing partition and creating a new one, with the following set of commands and parameters. When asked for input, to accept the default value, just press enter without entering anything.

    d    # delete the root partition
    n    # create a new partition
    p    # the new partition will be a primary partition
    <accept the default partition number>
    8192 # the new partition must begin on the same sector
    # as the old partition, which is usually 8192 - verify this!
    <accept the default size, which fits the whole device>
    w    # write the changes to the partition table

Finally, the filesystem can be resized to fill the complete enlarged partition:

    sudo resize2fs /dev/mmcblk1p1

## Network Setup

1. Connect to the board via USB with the same credentials as in step 4 of [Flashing the Image to Beaglebone](#flashing-the-image-to-beaglebone).

        ssh ubuntu@192.168.7.2

2. Set the default gateway

        sudo route add default gw 192.168.7.1

3. Uninstall the `connman` package, since we will be using a static network configuration through the `/etc/network/interfaces` file.

        sudo apt-get remove connman

4. Set up the nameserver for the DNS resolver. A usual safe choice is the Google nameserver located at `8.8.8.8`. For the arena in the Artificial life lab in Graz, use `143.50.56.25`.

        sudo su
        rm /etc/resolv.conf
        echo "nameserver 143.50.56.25" > /etc/resolv.conf

5. Add the following into `/etc/network/interfaces`. Replace _xy_ with your board number (e.g. `08`, `11`).

        # The primary network interface
        auto eth0
        iface eth0 inet static
            address 10.42.0.1xy
            netmask 255.255.255.0
            gateway 10.42.0.1
            dns-nameservers 143.50.56.25
        # Example to keep the MAC address between reboots
        #hwaddress ether DE:AD:BE:EF:CA:FE

Exit the root shell with `exit` or by pressing Ctrl-D.

6. Create the ___assisi___ user with a password ___assisi___ and add it to the necessary groups. Leave all fields (except the password) empty when creating a user.

        sudo adduser assisi
        sudo usermod -a -G assisi,adm,kmem,dialout,cdrom,floppy,sudo,audio,dip,video,plugdev,users,netdev,i2c,admin,spi,systemd-journal,weston-launch,xenomai assisi

7. Update `/etc/hosts`.

  Change the local name to _bbg-0xy_ (replace _arm_ with _bbg-0xy_), with _xy_ being the board number, same as above.
  Next, add the following hosts. It is also advisable to add them on your machine, and to include an entry for your machine as well (on both the Beaglebones and your machine).

      # CASUs
      10.42.0.1   control-workstation
      10.42.0.5   damjan-workstation
      10.42.0.51  thaus
      10.42.0.99  marsela
      10.42.0.101 bbg-001
      10.42.0.102 bbg-002
      10.42.0.103 bbg-003
      10.42.0.104 bbg-004
      10.42.0.105 bbg-005
      10.42.0.106 bbg-006
      10.42.0.107 bbg-007
      10.42.0.108 bbg-008
      10.42.0.109 bbg-009
      10.42.0.110 bbg-010
      10.42.0.111 bbg-011
      10.42.0.112 bbg-012
      10.42.0.113 bbg-013
      10.42.0.114 bbg-014
      10.42.0.115 bbg-015
      10.42.0.116 bbg-016
      10.42.0.253 bbg-airflow

8. Change the local hostname in `/etc/hostname` from _arm_ to _bbg-0xy_, with _xy_ being the board number, same as above.

9. Reboot the board.

10. Optionally, to setup passwordless login, execute on the host machine:

        ssh assisi@bbg-0xy mkdir -p .ssh
        cat .ssh/id_rsa.pub | ssh assisi@bbg-0xy 'cat >> .ssh/authorized_keys'

 You can also install the `clusterssh` package, which enables you to simultaneously connect and send keystrokes to multiple beaglebones. After installing, create a file named `clusters` in `~/.clusterssh`, and define a cluster. For example, to define a cluster named `bbs` consisting of 3 Beaglebones, add the following line:

      bbs 10.42.0.110 10.42.0.111 bbg-012

 To connect to all machines in the cluster bbs, invoke `cssh bbs`.

## Firmware Installation

### Firmware and Hardware Dependencies

1. Connect to the BBG. Install the updates and packages. Temporarily ignore the locale warnings (fix [here](#locale-warnings)).

        sudo apt-get update
        sudo apt-get install resolvconf libzmq3-dev libprotobuf-dev libyaml-cpp-dev protobuf-compiler libboost-all-dev cmake python python-zmq python-protobuf python-yaml python-pygraphviz python-sphinx fabric

2. Enable i2c bus 1.

      a. Download the .dts from [github](https://github.com/jadonk/cape-firmware/blob/master/arch/arm/boot/dts/BB-I2C1-00A0.dts). Copy it to the Beaglebone and compile.

        scp BB-I2C1-00A0.dts assisi@bbg-0xy:/home/assisi
        dtc -O dtb -o BB-I2C1-00A0.dtbo -b 0 -@ BB-I2C1-00A0.dts

      b. Move the file to `/lib/firmware` on the BBG

        cd /home/assisi
        mv BB-I2C1-00A0.dtbo /lib/firmware

      c. Find and edit the following two lines in `/boot/uEnv.txt` (one beginning with `cmdline=coherent_pool`, the other with `#cape_enable=bone`):

        cmdline=coherent_pool=1M quiet cape_universal=enable
        (...)
        ##Example v4.1.x
        #cape_disable=bone_capemgr.disable_partno=
        cape_enable=bone_capemgr.enable_partno=BB-I2C1

      e. Run the following script to apply the changes.

        /opt/scripts/tools/developers/update_initrd.sh

### Installation

1. Install assisipy

        sudo pip install assisipy

2. Download the Beaglebone firmware from Github:

        cd /home/assisi/
        mkdir firmware
        mkdir firmware/log
        mkdir firmware/old-log
        mkdir firmware/archives
        git clone https://github.com/assisi/casu-beaglebone
        cd casu-beaglebone
        git remote set-url origin https://github.com/assisi/casu-beaglebone
        git submodule update --init

3. Compile the firmware

        cd /home/assisi/casu-beaglebone
        mkdir build
        cd build
        cmake ..
        make
        cp beaglebone2pic/casu-fw /home/assisi/firmware
        cp ../config/bbg-0xy/casu-00* /home/assisi/firmware

4. Update `rc.local`.

        #!/bin/sh -e
        #
        # rc.local
        #
        # This script is executed at the end of each multiuser runlevel.
        # Make sure that the script will "exit 0" on success or any other
        # value on error.
        #
        # In order to enable or disable this script just change the execution
        # bits.
        #
        # By default this script does nothing.

        # Sync with real-time clock
        #source  /usr/share/rtc_ds1307/clock_init.sh

        sleep 0.25

        i2cset -y 2 0x70 0x00 0xF

        exit 0

### Setting up NTP Update and Firmware Launch on Startup

The following steps will setup the Beaglebone to update date and time on startup using an internet NTP server. When this is successfully completed, the CASU firmware is initialized, which ensures that all CASU logs have precise time information.

1. Make sure that the packages `ntp` and `ntpdate` are not installed.

        sudo apt purge ntp ntpdate

2. Install the `systemd-timesyncd-wait.service` addon for systemd, which provides a service that enters the finished state only when `systemd-timesyncd` successfully synchronizes the time and date with an NTP server.

        sudo apt install golang
        cd /home/assisi
        git clone https://github.com/assisi/systemd-timesyncd-wait
        cd systemd-timesyncd-wait
        make
        sudo make install

 Afterwards, you can remove the Go language packages to save space:

        sudo apt remove golang
        sudo apt autoremove

3. Create a service file named `setdate-initfirmware.service` in `/etc/systemd/system`, with the following contents

        [Unit]
        Description=Waits for time and date to be set, starts CASU firmware
        After=network-online.target systemd-timesyncd-wait.service
        Wants=network-online.target systemd-timesyncd-wait.service

        [Service]
        Type=forking
        ExecStart=/home/assisi/.fw_run.sh

        [Install]
        WantedBy=multi-user.target

4. Run the following commands to install the new service.

        sudo systemctl daemon-reload
        sudo systemctl enable setdate-initfirmware.service

5. Create a script file called `/home/assisi/.fw_run.sh`, with the following contents. Adjust the connected CASUs by modifying the `USE_CASUS` list.

        #!/bin/bash

        USE_CASUS=(1 2 3 4)

        echo "$(date) configured CASUs: ${USE_CASUS[@]}"

        for i in "${USE_CASUS[@]}"; do
          echo "Starting CASU #$i"
          /home/assisi/firmware/casu-fw /home/assisi/firmware/casu-00$i.fbc > /dev/null 2>&1 &
          sleep 0.25
        done

        echo "Launched PIDs:"
        echo "$(pgrep casu)"

        exit 0

6.  Set the executable bit on the script.

        chmod +x /home/assisi/.fw_run.sh

### Setting up CASU Firmware Status Display on Startup

It can be useful to add a status display that is run at the beginning of the ssh session, which helps verify that the correct time and date have been set and that the CASU firmware has been launched. To do so, open `/home/assisi/.bashrc` and append the following to the end of the file:

    # print a CASU firmware status message if in login shell
    if shopt -q login_shell
    then
      watch -n0.1 "
        date
        echo 'PIDs of CASU firmware loaded on $HOSTNAME:'
        pgrep casu
        echo 'Press CTRL-C to dismiss'
      "
    fi

## Locale Warnings

If a warning like the following shows up during package installation or similar operations:

      perl: warning: Setting locale failed.
      perl: warning: Please check that your locale settings:
        LANGUAGE = (unset),
        LC_ALL = (unset),
        LC_TIME = "hr_HR.UTF-8",
        LC_MONETARY = "hr_HR.UTF-8",
        LC_ADDRESS = "hr_HR.UTF-8",
        LC_TELEPHONE = "hr_HR.UTF-8",
        LC_NAME = "hr_HR.UTF-8",
        LC_MEASUREMENT = "hr_HR.UTF-8",
        LC_IDENTIFICATION = "hr_HR.UTF-8",
        LC_NUMERIC = "hr_HR.UTF-8",
        LC_PAPER = "hr_HR.UTF-8",
        LANG = "en_US.UTF-8"
          are supported and installed on your system.
      perl: warning: Falling back to a fallback locale ("en_US.UTF-8").
      locale: Cannot set LC_ALL to default locale: No such file or directory
      Preconfiguring packages ...
      (Reading database ... 47720 files and directories currently installed.)
      Removing connman (1.32-git20160418-0rcnee1~bpo1604+20160418+1) ...
      Selecting previously unselected package resolvconf.
      (Reading database ... 47700 files and directories currently installed.)
      Preparing to unpack .../resolvconf_1.78ubuntu2_all.deb ...
      Unpacking resolvconf (1.78ubuntu2) ...
      Processing triggers for systemd (229-4ubuntu8) ...
      Processing triggers for ureadahead (0.100.0-19) ...
      Setting up resolvconf (1.78ubuntu2) ...
      locale: Cannot set LC_ALL to default locale: No such file or directory
      Processing triggers for resolvconf (1.78ubuntu2) ...
      locale: Cannot set LC_ALL to default locale: No such file or directory

Set locale settings as follows and re-login:

    sudo sh -c "echo 'LANG=en_US.UTF-8\nLC_ALL=en_US.UTF-8' > /etc/default/locale"

## Building the Linux Kernel

It can be beneficial to install a newer Linux Kernel on the Beaglebone. To do this, run the following commands on your workstation:

    git clone https://github.com/RobertCNelson/ti-linux-kernel-dev.git
    cd ti-linux-kernel-dev
    git checkout origin/ti-linux-4.4.y -b tmp
    ./build_deb.sh

Copy the resulting `linux-firmware-...deb`, `linux-image...deb`, and `linux-libc...deb` files to the Beaglebone (e.g. `scp -r folder_with_deb_files assisi@bbg-0xy`. On the Beaglebone, install the new kernel with the command `sudo dpkg -i *.deb` while in the directory where the copied `.deb` packages are located.

## Enabling the RTC Clock

__Warning: currently, we do not use the RTC clock as it causes issues with the I2C bus. Following the instructions above will ensure that the date and time is set from internet on boot.__

Obtain a 3V CR1220 battery and insert it into the slot. Make sure you have internet access (ping www.google.com). Run `i2cdetect` to check if the Beaglebone detects the RTC.

    sudo su
    i2cdetect -y -r 2

The output should be as below. If there are only two "-" at field 68, enable i2c bus as in step two of [Firmware and Hardware Dependencies](#firmware-and-hardware-dependencies).

    0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:          -- -- -- -- -- -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    50: -- -- -- -- UU UU UU UU -- -- -- -- -- -- -- --
    60: -- -- -- -- -- -- -- -- UU -- -- -- -- -- -- --
    70: 70 -- -- -- -- -- -- --

Run the following.

    echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-2/new_device
    hwclock -r -f /dev/rtc1

The output should be:

    Mon 01 Jan 2000 HH:MM:SS PM UTC  .000000 seconds

Also check:

    hwclock -r -f /dev/rtc

The output should be:

    x 11 Feb 2016 HH:MM:SS PM UTC  .000000 seconds

Also check:

    hwclock -r -f /dev/rtc0

This should output (or other date, not the one of today):

    x 11 Feb 2016 HH:MM:SS PM UTC  .000000 seconds

Run the following to check the system date and time:

    timedatectl

Wait until the output of the following is the current date and time

    hwclock -r -f /dev/rtc0

When you reached the correct time, write the current time:

    hwclock -w -f /dev/rtc1

To run the clock update at boot, create a script:

    mkdir /usr/share/rtc_ds1307
    nano /usr/share/rtc_ds1307/clock_init.sh

Paste the following content into the script:

    #!/bin/bash
    sleep 5
    echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-2/new_device
    hwclock -s -f /dev/rtc1
    hwclock -w

Paste the following into `/lib/systemd/system/rtc-ds1307.service`

    [Unit]
    Description=DS1307 RTC Service
    [Service]
    Type=simple
    WorkingDirectory=/usr/share/rtc_ds1307
    ExecStart=/bin/bash clock_init.sh
    SyslogIdentifier=rtc_ds1307
    Wants=systemd-timesyncd-wait.service
    After=systemd-timesyncd-wait.service

    [Install]
    WantedBy=multi-user.target

To install the service, run

    systemctl enable rtc-ds1307.service

Finally, reboot and the RTC should be working.
