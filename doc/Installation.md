# Initialization the workspace for panda_object_detection

## some dependencies
```
sudo apt-get install libsdl1.2-dev libosmesa6-dev
```

## realsense_camera
```
# for dependency - librealsense
wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
bash ./enable_kernel_sources.sh
sudo apt-get update
sudo apt-get install linux-image-generic-lts-xenial
sudo reboot
sudo apt-get dist-upgrade
sudo apt-get install ros-kinetic-realsense-camera
```
If you want to use realsense2_camera, just refer to the [Realsense Installation](https://github.com/robinloujun/robina_object_detection/blob/master/doc/Realsense_Initialization.md)

## kinect_camera
```
sudo apt-get install ros-kinetic-openni-camera ros-kinetic-openni-launch
```

## ecto

ORK is built on top of [ecto](http://plasmodic.github.io/ecto/) which is a 
lightweight hybrid C++/Python framework for organizing computations as directed 
acyclic graphs.

```
sudo apt-get install libboost-python-dev libboost-filesystem-dev libboost-system-dev \
libboost-thread-dev python-setuptools python-gobject python-gtk2 graphviz doxygen \
python-sphinx 
```
Download the package and build the workspace (in src)
```
git clone http://github.com/plasmodic/ecto.git
cd ..
catkin_make
```

## ORK
Install the [ORK](https://wg-perception.github.io/object_recognition_core/) from the ROS repositories
```
sudo apt-get install ros-kinetic-object-recognition-core
```

Download & Build from Source  
```
sudo apt-get install libopenni-dev ros-kinetic-catkin ros-kinetic-ecto* \
ros-kinetic-opencv-candidate ros-kinetic-moveit-msgs
source /opt/ros/kinetic/setup.sh
```
Download the package and build the workspace (in src)
```
git clone http://github.com/wg-perception/object_recognition_core
git clone http://github.com/wg-perception/capture
git clone http://github.com/wg-perception/reconstruction
git clone http://github.com/wg-perception/linemod
git clone http://github.com/wg-perception/ork_renderer
git clone http://github.com/wg-perception/tabletop
git clone http://github.com/wg-perception/tod
git clone http://github.com/wg-perception/transparent_objects
git clone http://github.com/wg-perception/object_recognition_msgs
git clone http://github.com/wg-perception/object_recognition_ros
git clone http://github.com/wg-perception/object_recognition_ros_visualization
cd ..
catkin_make
```

## Install Couch Db
```
sudo apt-get install couchdb 
```
Test with 
```
curl -X GET http://localhost:5984
```
Install couchapp 
```
pip install -U couchapp
```
Installs the visualizer on the DB 
```
rosrun object_recognition_core push.sh
```

p.s.: Configuring the database  
DB implementations provided by default
- CouchDb: `{'type': 'CouchDB', 'root': 'http://localhost:5984', 'collection': 'object_recognition'}`
- Filesystem: `{'path': '/tmp', 'type': 'filesystem', 'collection': 'object_recognition'}`
- Empty (only for testing): `{'type': 'empty'}`   
