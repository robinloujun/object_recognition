project(ObjRecRANSACRun)
cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

IF(OBJREC_USE_CUDA)
  set(OBJREC_CUDA_DEVICE_ID 1 CACHE STRING "Which GPU device to use when running")
  add_definitions(-DUSE_CUDA)
  add_definitions(-DCUDA_DEVICE_ID=${OBJREC_CUDA_DEVICE_ID})

  set(CUDA_TOOLKIT_ROOT_DIR $ENV{CUDA_TOOLKIT_ROOT_DIR})
  FIND_PACKAGE(CUDA REQUIRED)

  add_definitions(-DOBJ_REC_RANSAC_PROFILE)

  #add_definitions(-DOBJ_REC_RANSAC_VERBOSE)
  #add_definitions(-DOBJ_REC_RANSAC_VERBOSE_1)
  #add_definitions(-DOBJ_REC_RANSAC_VERBOSE_2)
ENDIF(OBJREC_USE_CUDA)

find_package( Boost COMPONENTS thread system REQUIRED )
find_package( PCL REQUIRED )
find_package( OpenMP REQUIRED )
find_package( VTK 6 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( PCL REQUIRED )
find_package( realsense2 )
find_package( glfw3 3.2 REQUIRED )
find_package( OpenGL REQUIRED )

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/include/vtk-6.2/
  /usr/local/cuda/include
  ${OpenCV_INCLUDE_DIRS}
  ./src
  "/home/robin/Libraries/librealsense/examples/")

link_directories(${PCL_LIBRARY_DIRS})
link_directories(
  ${VTK_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  /usr/local/cuda/lib
  ${OpenCV_LIB_DIR}
  )
add_definitions(${PCL_DEFINITIONS})

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
  If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)

IF(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found. OpenCV is not installed or cmake can not find it. Install OpenCV first and then try again.
  If OpenCV is installed but cmake can not find it, set the OPENCV_DIR entry to the folder which contains the file OpenCVConfig.cmake")
ENDIF(NOT OpenCV_FOUND)

IF(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
ENDIF()

IF(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message (STATUS "Found OpenMP")
ENDIF(OPENMP_FOUND)

IF(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it \n\n")
ENDIF(NOT realsense2_FOUND)

add_subdirectory(src/BasicToolsL1)
add_subdirectory(src/BasicTools)
add_subdirectory(src/ObjRecRANSAC)
add_subdirectory(src/VtkBasics)
#add_subdirectory(src/Greedy)

add_executable(capture capture.cpp)
target_link_libraries(capture ${PCL_LIBRARIES} ${OpenCV_LIBS} realsense2 glfw ${HEADERS} ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics ${VTK_LIBRARIES} rt)

install(DIRECTORY src/
  DESTINATION include
  PATTERN "*.cpp" EXCLUDE
  PATTERN "*.cu" EXCLUDE
  PATTERN "*.c" EXCLUDE
  )

install(DIRECTORY data
  DESTINATION share/ObjRecRANSAC)

install(FILES package.xml
  DESTINATION share/ObjRecRANSAC)

message(STATUS "VTK_VERSION: " ${VTK_VERSION})

