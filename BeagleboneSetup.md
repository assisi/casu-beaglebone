 	# Beaglebone environment setup instructions 

These instructions are intended for Beaglebone Green installation and setup. The steps are described for installation from a work station running on Ubuntu 14.04 (but should also work on 16.04).

## Ubuntu 16.04 installation

If you already have a prepared microSD card, skip to [Flash image to Beaglebone](#flash-image-to-beaglebone).

### Ubuntu 16.04 image

Otherwise, [get a prebuilt image](http://elinux.org/BeagleBoardUbuntu#Demo_Image) for BeagleBone/BeagleBone Black. Do not download a flasher image. Follow the steps described on the link: 

- download .tar file
- verify and unpack image
- locate your SD card (`fdisk -l` is useful)
- format SD card, install image on a partition preferably of the same size as BeagleBone flash memory (4 GB at the moment) 

### Flash image to Beaglebone

1. Insert microSD card into Beaglebone microSD slot
2. While pressing the USER button next to the slot, plug the board to your station via usb (micro-USB - USB cable). Continue pressing the button while four blue LEDs light up and start blinking "randomly"
3. Connect your station to the wired connection (_Seeed BeagleBoneGreen_)
4. Connect to the board from terminal under default user _ubuntu_. The default board IP is 192.168.7.2. The default password for the image is _temppwd_. 

        ssh ubuntu@192.168.7.2
Upon login, you should see that the system has booted on Ubuntu 16.04
4. Determine path to SD card and local flash memory using `fdisk -l` and `df`. Comparing memory sizes can be helpful (if different). Usually, the card is `mmcblk0`, and the local memory `mmcblk1`.
6. Copy the image from SD to local flash with `dd`. __Replace paths if they differ in your case.__ `if` is the source, and `of` is the destination. If the SD card only has a partition with the image, it is safe to flash the whole card, and not just a partition.

        sudo dd if=/dev/mmcblk0 of=/dev/mmcblk1 bs=4M status=progress
If your card is larger than the BeagleBone flash memory, an error will be shown after 3.6Gb of data is transferred. It is safe to ignore this error. 
7. Reboot the board without SD card (unplug from power, unplug card, plug to power)

### Network setup

1. Connect to the board via USB with the same credentials as in step 4 of [Flash image to Beaglebone](#flash-image-to-beaglebone).

        ssh ubuntu@192.168.7.2
2. Set default gateway
        
        sudo route add default gw 192.168.7.1
3. Uninstall connman services

        sudo apt-get remove connman
4. Explanation needed here. Arena location etc.

        sudo su
        rm /etc/resolv.conf
        echo "nameserver 143.50.56.25" > /etc/resolv.conf
5. Add the following code into the `/etc/network/interfaces`. Replace the _xy_ with your board number.

        # The primary network interface
        auto eth0
        iface eth0 inet static
            address 10.42.0.1xy 
            netmask 255.255.255.0
            gateway 10.42.0.1 
            dns-nameservers 143.50.56.25      
        # Example to keep MAC address between reboots
        #hwaddress ether DE:AD:BE:EF:CA:FE
Exit the root with `exit`.
6. Create _assisi_ user with a password _assisi_ and add it to necessary groups. Leave all field except password empty when creating a user.

        sudo adduser assisi (password assisi, other empty)
        sudo usermod -a -G assisi,adm,kmem,dialout,cdrom,floppy,sudo,audio,dip,video,plugdev,users,netdev,i2c,admin,spi,systemd-journal,weston-launch,xenomai assisi
7. Update `/etc/hosts`. Change local name to _bbg-0xy_ (replace _arm_ with _bbg-0xy_). Add following hosts to `/etc/local`: (TODO: check if airflow name is correct)

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
8. Change local hostname in `\etc\hostname` from _arm_ to _bbg-0xy_.
8. Reboot the board. 
9. Optionally, setup passwordless login from host, on host machine:

        ssh assisi@bbg-0xy mkdir -p .ssh
        cat .ssh/id_rsa.pub | ssh assisi@bbg-00x 'cat >> .ssh/authorized_keys'

### Firmware & Hardware Dependencies

1. Connect to your BBG. Install updates and packages. Temporarily ignore locale warnings.

        sudo apt-get update
        sudo apt-get install resolvconf ntp libzmq3-dev libprotobuf-dev libyaml-cpp-dev protobuf-compiler libboost-all-dev cmake python python-zmq python-protobuf python-yaml python-pygraphviz python-sphinx fabric
2. Enable i2c bus 1. 
    a. Download a compiled .dtbo file from *** (TODO: find location to store file). 
    b. Copy this file to BeagleBone: from local host, find downloaded file and secure-copy it to BeagleBone.

        scp BB-I2C1-00A0.dtbo assisi@bbg-0xy:/home/assisi 
    c. Move file to `/lib/firmware` on BBG

        cd /home/assisi
        mv BB-I2C1-00A0.dtbo /lib/firmware
    d. Find and edit following two lines in `/boot/uEnv.txt` (one beginning with `cmdline=coherent_pool`, other with `#cape_enable=bone`):


        cmdline=coherent_pool=1M quiet cape_universal=enable
        (...)
        ##Example v4.1.x
        #cape_disable=bone_capemgr.disable_partno=
        cape_enable=bone_capemgr.enable_partno=BB-I2C1
    e. Run the following script to update the changes.

        /opt/scripts/tools/developers/update_initrd.sh

### Firmware Installation

1. Install assisipy

        sudo pip install assisipy
2. Download BeagleBone firmware from github

        cd
        mkdir firmware
        mkdir firmware/log
        git clone https://github.com/assisi/casu-beaglebone
        git remote set-url origin https://github.com/assisi/casu-beaglebone
        git submodule update --init
3. Compile firmware

        cd casu-beaglebone
        mkdir build
        cd build
        cmake ..
        make
        cp beaglebone2pic/casu-fw /home/assisi/firmware
        cp ../config/bbg-0xy/casu-00* /home/assisi/firmware
4. Update `rc.local` to run firmware at boot

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

        # Start CASU firmware
        sleep 0.25

        i2cset -y 2 0x70 0x00 0xF

        sleep 0.25

        /home/assisi/firmware/casu-fw /home/assisi/firmware/casu-001.fbc > /dev/null 2>&1 &

        sleep 0.25

        /home/assisi/firmware/casu-fw /home/assisi/firmware/casu-002.fbc > /dev/null 2>&1 &

        sleep 0.25

        /home/assisi/firmware/casu-fw /home/assisi/firmware/casu-003.fbc > /dev/null 2>&1 &

        sleep 0.25

        /home/assisi/firmware/casu-fw /home/assisi/firmware/casu-004.fbc > /dev/null 2>&1 &

        sleep 0.25

        exit 0
