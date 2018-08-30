# Initialization for the RealSense Package

ROS Wiki for [RealSense](http://wiki.ros.org/RealSense), 
[realsense_camera](http://wiki.ros.org/realsense_camera)

**Intel RealSense packages** enable the use of Intel RealSense R200, F200, **SR300** and
D400 cameras with ROS. 

The first thing is to check the ROS version using ```rosversion -d```

It should be **kinetic** for the project

- [Installation Prerequisites](http://wiki.ros.org/librealsense#Installation_Prerequisites)
: system will need to be configured to enable the downloading of 
kernel source files. 

- Enable Kernel Sources to enable the downloading of kernel sources on the
system (with Script)  
  `wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
  bash ./enable_kernel_sources.sh`

- Update to the Xenial kernel  
  ```
  sudo apt-get install linux-image-generic-lts-xenial
  sudo reboot
  ```

- Install the librealsense Package  
  `sudo apt-get install 'ros-kinetic-librealsense'`

- Install the ROS Debian Package  
  `sudo apt-get install 'ros-kinetic-realsense-camera'`

# Initialization for the Intel RealSense SDK 2.0

[**Intel RealSense SDK 2.0**]
(https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) 
is a cross-platform library for Intel RealSense depth cameras (D400 series and the SR300).

- Add Intel server  to the list of repositories :  
`echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list`  
It is recommended to backup `/etc/apt/sources.list.d/realsense-public.list` file in case of an upgrade.

- Register the server's public key :  
`sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE`  
- Refresh the list of repositories and packages available :  
`sudo apt-get update`  

- In order to run demos install:  
  `sudo apt-get install librealsense2-dkms`  
  `sudo apt-get install librealsense2-utils`  
  The above two lines will deploy librealsense2 udev rules, kernel drivers, runtime library and executable demos and tools.
  Reconnect the Intel RealSense depth camera and run: `realsense-viewer`  

- Developers shall install additional packages:  
  `sudo apt-get install librealsense2-dev`  
  `sudo apt-get install librealsense2-dbg`  
  With `dev` package installed, you can compile an application with **librealsense** using `g++ -std=c++11 filename.cpp -lrealsense2` or an IDE of your choice.

    Verify that the kernel is updated :    
    `modinfo uvcvideo | grep "version:"` should include `realsense` string
