This function is based on the [ObjRecRANSAC](https://github.com/tum-mvp/ObjRecRANSAC) using a RealSense RGBD camera.
- RANSAC variant for 3D object recognition in occluded scenes 

# Dependencies

- [GLFW3](http://www.glfw.org/docs/latest/compile.html)
- [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)  
    For RealSense capture
- PCL - Version 1.7 ( the latest version is 1.8.1 )
- VTK - Version 6.3.0 ( the latest version is 8.1.1)
- [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

# Compile

```
cd /Path/To/RANSAC
mkdir build
cd build
cmake ..
make -j[n] # Allow n jobs at once; infinite jobs without argument -j.
```
