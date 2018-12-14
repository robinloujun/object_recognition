# robina_object_detection

This is a workspace for object recognition on a table. It is based on the ```Object Recognition Kitchen (ORK)```
, which is a project started at Willow Garage for object recognition.

## Installation

This software is tested under 64 Bit Ubuntu Linux 16.04.  
You need a RealSense or Kinect RGBD camera and follow the [Installation](https://github.com/robinloujun/object_recognition/blob/master/doc/Installation.md#initialization-the-workspace-for-panda_object_detection)

## Calibration

Get the dependencies and compile the driver, there is a [guide](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
```
rosdep install camera_calibration
```
run the calibration
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/rgb/image_raw camera:=/camera/rgb
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/ir/image_raw camera:=/camera/ir
```

**[For Kinect camera]**  
The calibration data would be automatically saved in ```~/.ros/camera_info```

**[For RealSense camera]**  
There is no rosservice ```/set_camera_info``` for realsense camera, add the arguement ```--no-service-check```   
The calibration data can only be saved to ```/tmp/calibrationdata.tar.gz``` instead of direct commit 
Copy the ost.yaml in the tar.gz file to ```~/.ros/camera_info``` and change the file name

edit the launch file of ```realsense_camera``` (sr300_nodelet_rgbd.launch) (sudo needed) and add the following lines
```
  <!-- camera calibration file url -->
  <arg name="rgb_camera_info_url" default="~/.ros/camera_info/sr300_rgb.yaml" />
  <arg name="ir_camera_info_url" default="~/.ros/camera_info/sr300_depth.yaml" />
```

## Usage of ORK_tabletop
### Using Realsense D435
```
ce robina_ork
roscore
roslaunch realsense2_camera rs_rgbd.launch
rosrun object_recognition_core detection -c  `rospack find object_recognition_tabletop`/conf/detection_d435.object.ros.ork
cd .rviz
rosrun rviz rviz -d object_detection.rviz
```
### Using Realsense SR300
```
ce robina_ork
roscore
roslaunch realsense_camera sr300_nodelet_rgbd.launch
rosrun object_recognition_core detection -c  `rospack find object_recognition_tabletop`/conf/detection_sr300.object.ros.ork
cd .rviz
rosrun rviz rviz -d object_detection.rviz
```
### Using Kinect
```
ce robina_ork
roscore
roslaunch openni_launch openni.launch
rosrun dynamic_reconfigure dynparam set /camera/driver image_mode 1
rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
rosrun object_recognition_core detection -c  `rospack find object_recognition_tabletop`/conf/detection.object.ros.ork
cd .rviz
rosrun rviz rviz -d kinect_test.rviz
```

## Usage of ORK_linemod
Refer to the usage of ORK_tabletop to launch the camera and check the result in rviz.

First run the training step, the kitchen will learn from the models from the Database.
```
rosrun object_recognition_core training -c `rospack find object_recognition_linemod`/conf/training.ork
```
Then replace the detecting function in tabletop with the following function regarding the camera.
```
rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork # for kinect
rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection_sr300.ros.ork
rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection_d435.ros.ork
```

## Edit the [DB](http://localhost:5984/or_web_ui/_design/viewer/index.html)

**Creating an object & adding the mesh in the DB**
```
rosrun object_recognition_core object_add.py -n <name_of_object> -d "<description>" --commit
rosrun object_recognition_core mesh_add.py <OBJECT_ID> <path_to_the_model> --commit
```
**Deleting an object**
```
rosrun object_recognition_core object_delete.py <OBJECT_ID> --commit
```
check the [objct in DB](http://localhost:5984/_utils/database.html?object_recognition/_design/objects/_view/by_object_name)

check the [uploaded mesh](http://localhost:5984/or_web_ui/_design/viewer/meshes.html)
