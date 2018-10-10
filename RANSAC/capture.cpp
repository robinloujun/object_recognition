#include <iostream>
#include <time.h>
#include <string>
#include <ctime>
#include <list>

// ObjRecRANSAC
#include <BasicTools/Vtk/VtkTransform.h>
#include <BasicTools/ComputationalGeometry/Algorithms/RANSACPlaneDetector.h>
#include <ObjRecRANSAC/ObjRecRANSAC.h>

// Include RealSense Cross Platform API
#include <librealsense2/rs.hpp>
#include "example.hpp"

// Include OpenCV API
#include <opencv2/opencv.hpp>

// For PCL
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>

// For VTK
#include <vtkSmartPointer.h>
#include <vtkPolyDataWriter.h>
#include <VtkBasics/VtkWindow.h>
#include <VtkBasics/VtkPolyData.h>
#include <VtkBasics/VtkPoints.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtk-6.2/vtkSTLReader.h>
#include <vtk-6.2/vtkOBJReader.h>
#include <vtk-6.2/vtkSTLWriter.h>
#include <vtkTriangleFilter.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;

#define REMOVE_PLANE // For ObjRecRANSAC preprocessing
//#define VISUALIZE_RANSAC // Need to be quitted at each iteration if set pos

void visualize(list<boost::shared_ptr<PointSetShape> >& detectedShapes, vtkPoints* scene, vtkPoints* background);
void objrecransac_main(vtkSmartPointer<vtkPolyData>& vtk_points);

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

float3 colors[] { { 0.8f, 0.1f, 0.3f },
                  { 0.1f, 0.9f, 0.5f },
};

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

vtkSmartPointer<vtkPolyData> points_to_vtk(const pcl_ptr& cloud)
{
    vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::pointCloudTovtkPolyData(*cloud, mesh);
    mesh -> GetPointData();
    cout << "VTK read " << mesh->GetNumberOfPoints() << " points" << endl;
    return mesh;
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_TEXTURE_2D);

    int color = 0;

    for (auto&& pc : points)
    {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

        glBegin(GL_POINTS);
        glColor3f(c.x, c.y, c.z);

        /* this segment actually prints the pointcloud */
        for (int i = 0; i < pc->points.size(); i++)
        {
            auto&& p = pc->points[i];
            if (p.z)
            {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
    }

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}

// Test if a folder exists
inline bool test_exist(const string &folder_name){
  struct stat buffer;
  return (stat (folder_name.c_str(), &buffer) == 0);
}

int main(int argc, char * argv[]) try
{
  clock_t start = clock();

  // Check the pcd & vtk root path
  /*
  string pcd_path = "/home/robin/panda_object_detection/6_capture/pcd/";
  string vtk_path = "/home/robin/panda_object_detection/6_capture/vtk/";
  if ( !test_exist(pcd_path) )
  {
    const char* mkdir_pcd_path  = pcd_path.c_str();
    mkdir(mkdir_pcd_path, 0777);
    cout << "Built a folder named pcd to save the pcd files in " << pcd_path << endl;
  }
  if ( !test_exist(vtk_path) )
  {
    const char* mkdir_vtk_path  = vtk_path.c_str();
    mkdir(mkdir_vtk_path, 0777);
    cout << "Built a folder named pcd to save the pcd files in " << vtk_path << endl;
  }
  */

  // Create a simple OpenGL window for rendering:
  window app(640, 480, "Robina Object Detection");
  // Construct an object to manage view state
  glfw_state app_state;
  // register callbacks to allow manipulation of the pointcloud
  register_glfw_callbacks(app, app_state);

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Start streaming with default recommended configuration
  pipe.start();

  while (app) // Application still alive?
  {
      // Wait for the next set of frames from the camera
      auto frames = pipe.wait_for_frames();

      auto depth = frames.get_depth_frame();

      // Generate the pointcloud and texture mappings
      points = pc.calculate(depth);

      auto color = frames.get_color_frame();
      auto pcl_points = points_to_pcl(points);

      // Get the vtkPolyData of point cloud
      //vtkSmartPointer<vtkPolyData> vtk_points = points_to_vtk(pcl_points);
      //objrecransac_main(vtk_points);

      // Write the point cloud into vtk files for later test (also too slow)
      /*
      vtkSmartPointer<vtkPolyDataWriter> vtk_writer = vtkSmartPointer<vtkPolyDataWriter>::New();
      vtk_writer->SetInputData(vtk_points);
      clock_t timestamp = (clock()-start)/CLOCKS_PER_SEC;
      string tmp_str = vtk_path + to_string(timestamp) + ".vtk";
      vtk_writer->SetFileName(tmp_str.c_str());
      cout << "vtk file " << tmp_str << " saved" << endl;
      vtk_writer->Update();
      */

      pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(pcl_points);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.0, 3.0); // unit: m
      pass.filter(*cloud_filtered);

      // Get the vtkPolyData of filtered point cloud
      vtkSmartPointer<vtkPolyData> vtk_points = points_to_vtk(cloud_filtered);
      objrecransac_main(vtk_points);

      std::vector<pcl_ptr> layers;
      layers.push_back(pcl_points);
      layers.push_back(cloud_filtered);

      /* Uncomment the lines if color frames needed
      // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
      if (!color)
          color = frames.get_infrared_frame();

      // Tell pointcloud object to map to this color frame
      pc.map_to(color);

      // Upload the color frame to OpenGL
      app_state.tex.upload(color);
      Uncomment the lines if color frames needed */

      // Draw the pointcloud
      draw_pointcloud(app.width(), app.height(), app_state, points);

      // Write the point cloud into PCD files for later test (but too slow)
      /*
      pcl::PCDWriter pcd_writer;
      clock_t timestamp = (clock()-start)/CLOCKS_PER_SEC;
      string tmp_str = pcd_path + to_string(timestamp) + ".pcd";
      cout << tmp_str <<endl;
      pcd_writer.write<pcl::PointXYZ>(tmp_str, *cloud_filtered, false);
       */
  }

  return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception & e)
{
  std::cerr << e.what() << std::endl;
return EXIT_FAILURE;
}

void objrecransac_main(vtkSmartPointer<vtkPolyData>& vtk_points)
{
  // Some parameters:
  // 'pairwidth' should be roughly half the extent of the visible object part. This means, for each
  // object point p there should be (at least) one point q (from the same object) such that
  // ||p - q|| <= 'pairwidth'.
  // TRADEOFF: smaller values allow for detection in occluded scenes but lead to more imprecise alignment.
  // Bigger values lead to better alignment but require large visible object parts.
  double pairWidth = 100.0;// in millimeter (since the models in this example are in millimeter - see below)
  // 'voxelsize' is the size of the leafs of the octree, i.e., the "size" of the discretization.
  // TRADEOFF: High values lead to less computation time but ignore object details.
  // Small values allow to better distinguish between objects, but will introduce more holes in the resulting
  // "voxel-surface" (especially for a sparsely sampled scene) and thus will make normal computation unreliable.
  // Processing time, of course, will increase with smaller voxel size.
  double voxelSize = 6.0; // in millimeter

  // Create the main object
  ObjRecRANSAC objrec(pairWidth, voxelSize, 0.5/*leave this one like this*/);

  // Some lists
  list<UserData*> userDataList; // Look inside the next function to see how to use this list
  list<vtkPolyDataReader*> readers; // This is just to delete the readers at the end
  // Load the models to the hash table. Look inside this function - it is important since you
  // will want to load your own models

  // Some additional parameters for the recognition
  // The desired success probability for object detection. The higher the value the more samples are
  // needed => the more computation time will expire.
  // TRADEOFF: clear.
  double successProbability = 0.99;
  // The "visibility" is the expected visible object part expressed as fraction of the hole object.
  // For example 0.1 means that 10% of the object surface is visible in the scene.
  // Note that the visibility can not be more than 0.5 since a typical scanning device can not see
  // more than the half of the object.
  // TRADEOFF: smaller values allow for a detection in occluded scenes but also lead to more
  // false positives since object hypotheses with small alignment with the scene will be accepted
  objrec.setVisibility(0.2);
  // The "relative object size" is the expected fraction of the scene points which belong to an object.
  // For example a value of 0.05 means that each object present in the scene will contain at
  // least 5% of all scene points.
  // TRADEOFF: lower values lead to more computation time and to higher success probability.
  objrec.setRelativeObjectSize(0.1);
  // This should equal the number of CPU cores
  objrec.setNumberOfThreads(8);

  // This list will contain the model instances which are detected in the scene. After the object detection has been
  // performed, use the 'getUserData()' method for each 'PointSetShape' in the list in order to know which is the
  // object you are currently considering.
  list<boost::shared_ptr<PointSetShape> > detectedShapes;

  // Now the (optional) scene pre-processing takes place followed by the object recognition.

  // Load the scene - for this example from the hard drive. In a real scenario it will come from a range scanner.
  // VERY IMPORTANT ASSUMPTIONS:
  // 1) All points have to have positive z coordinates, i.e., all scene points are lying on one side (the "positive" side)
  //    of the scanning device and
  // 2) the origin of the point cloud has to be "in front" of the points, i.e., the rays passing through the origin have to
  //    meet the "outer" part of the objects surface.
  //
  // If 1) doesn't hold, i.e., your scanner provides you with points with negative z coordinates, do *not* just translate
  // the point cloud such that the points get positive z. If you do that, 2) will not hold. Instead, rotate the cloud by
  // 180° around the x or y axis.
  //vtkPolyDataReader* sceneReader = vtkPolyDataReader::New();
  //vtkOBJReader* sceneReader = vtkOBJReader::New();

  // Load the vtk scene model
  vtkPoints* scene = vtk_points->GetPoints() ;
  int num_points = scene->GetNumberOfPoints();
  printf("Scene has %i point(s).\n", num_points);
  fflush(stdout);

  if (num_points!=0)
  {
      // This is an optional pre-processing: if all objects are on a table and if that table occupies
      // a significant portion of the scene it would make sense to detect the plane and remove all its
      // points and the points below the plane.
    #ifdef REMOVE_PLANE
      // Some parameters for the plane removal
      double planeThickness = 10.0; // Since real data is noisy the plane is not infinitely thin
      double relNumOfPlanePoints = 0.2; // At least 20% of the scene points belong to the plane
      RANSACPlaneDetector planeDetector;
      // Perform the plane detection
      planeDetector.detectPlane(scene, relNumOfPlanePoints, planeThickness);
      // Check the orientation of the detected plane normal
      if ( planeDetector.getPlaneNormal()[2] > 0.0 )
        planeDetector.flipPlaneNormal();

      // Get the points above the plane (the scene) and the ones below it (background)
      scene = vtkPoints::New(VTK_DOUBLE);
//      scene = vtk_points->GetPoints() ;
      vtkPoints* background = vtkPoints::New(VTK_DOUBLE);
      planeDetector.getPointsAbovePlane(scene, background);
    #else
      vtkPoints* background = NULL;
    #endif

      // Perform the object recognition. You can call this method arbitrary often (perhaps each time with a new scene).
      // However, do NOT forget to free the memory the pointers in the list 'detectedShapes' are pointing to after EACH call!

      objrec.doRecognition(scene, successProbability, detectedShapes);
      printf("%lf seconds elapsed.\n", objrec.getLastOverallRecognitionTimeSec());
    #ifdef VISUALIZE_RANSAC
      visualize(detectedShapes, scene, background);
    #endif

      // Cleanup
      // Destroy the 'UserData' objects
      for ( list<UserData*>::iterator it = userDataList.begin() ; it != userDataList.end() ; ++it ) delete *it;
      // Destroy the readers
      for ( list<vtkPolyDataReader*>::iterator it = readers.begin() ; it != readers.end() ; ++it ) (*it)->Delete();
      // Destroy the scene reader
      // sceneReader->Delete();

    #ifdef REMOVE_PLANE
      background->Delete();
      scene->Delete();
    #endif
  }
}

//===================================================================================================================

void loadModels(ObjRecRANSAC& objrec, list<UserData*>& userDataList, list<vtkPolyDataReader*>& readers)
{
  char fileName[1024];
  // Derive the class 'UserData' if you want to save some specific information
  // about each model. When you load a model in the library you have to pass a 'UserData'-pointer
  // to the method 'addModel()'. If the corresponding model is detected in the scene, you can use
  // the 'UserData'-pointer which is returned by the recognition method, in order to know which
  // model has been detected.
  UserData* userData;

  // FIRST MODEL
  // Create a user object.
  userData = new UserData();
  userData->setLabel("Brush"); // Just set an 'Amicelli' label
  // Load the model
  sprintf(fileName, "../../2_models/Brush_top.stl");
  //vtkPolyDataReader* reader1 = vtkPolyDataReader::New();
  vtkSTLReader* reader1 = vtkSTLReader::New();
  reader1->SetFileName(fileName);
  reader1->Update();
  // Add the model to the model library
  objrec.addModel(reader1->GetOutput(), userData);
  // Save the user data and the reader in order to delete them later (outside this function)
  userDataList.push_back(userData);
  readers.push_back((vtkPolyDataReader*)reader1);

//  // SECOND MODEL
//  // Create a user object
//  userData = new UserData();
//  userData->setLabel("Panda_Cup");
//  // Load the model
//  sprintf(fileName, "../../2_models/Panda-Cup.stl");
////  vtkPolyDataReader* reader2 = vtkPolyDataReader::New();
//  vtkSTLReader* reader2 = vtkSTLReader::New();
//  reader2->SetFileName(fileName);
//  reader2->Update();
//  // Add the model to the model library
//  objrec.addModel(reader2->GetOutput(), userData);
//  // Save the user data and the reader in order to delete them later (outside this function)
//  userDataList.push_back(userData);
//  readers.push_back((vtkPolyDataReader*)reader2);

//  // THIRD MODEL
//  // Create a user object
//  userData = new UserData();
//  userData->setLabel("Pillow");
//  // Load the model
//  sprintf(fileName, "../../2_models/Pillow.stl");
////  vtkPolyDataReader* reader3 = vtkPolyDataReader::New();
//  vtkSTLReader* reader3 = vtkSTLReader::New();
//  reader3->SetFileName(fileName);
//  reader3->Update();
//  // Add the model to the model library
//  objrec.addModel(reader3->GetOutput(), userData);
//  // Save the user data and the reader in order to delete them later (outside this function)
//  userDataList.push_back(userData);
//  readers.push_back((vtkPolyDataReader*)reader3);

}

//=========================================================================================================================

void visualize(list<boost::shared_ptr<PointSetShape> >& detectedShapes, vtkPoints* scene, vtkPoints* background)
{
  printf("Visualizing ...\n");

  VtkWindow vtkwin(0, 0, 1000, 800);
    vtkwin.setCameraPosition(131.220071, -240.302073, -162.992888);
    vtkwin.setCameraFocalPoint(-48.026838, -54.679381, 787.833180);
    vtkwin.setCameraViewUp(-0.044383, 0.978898, -0.199470);
    vtkwin.setCameraViewAngle(30.000000);
    vtkwin.setWindowSize(480, 320);

  list<VtkPolyData*> transformedModelList;

  // Visualize the detected objects (important to look inside this loop)

  int color = 1;

  for ( list<boost::shared_ptr<PointSetShape> >::iterator it = detectedShapes.begin() ; it != detectedShapes.end() ; ++it )
  {
    cout << "before loop" << endl;
    boost::shared_ptr<PointSetShape>  shape = (*it);
    // Which object do we have (and what confidence in the recognition result)
    if ( shape->getUserData() )
      printf("\t%s, confidence: %lf\n", shape->getUserData()->getLabel(), shape->getConfidence());

    // Allocate memory for a homogeneous matrix
    double **mat4x4 = mat_alloc(4, 4);
    // Get the estimated rigid transform
    shape->getHomogeneousRigidTransform(mat4x4);

    // Transform the model instance using the estimated rigid transform
    vtkTransformPolyDataFilter *transformer = vtkTransformPolyDataFilter::New();
      transformer->SetInputData(shape->getHighResModel());
      VtkTransform::mat4x4ToTransformer((const double**)mat4x4, transformer);

    // Visualize the transformed model
    VtkPolyData* transformedModel = new VtkPolyData(transformer->GetOutput());
                  transformedModel->setColor(1.0, 0.55, 0.05);
      //transformedModel->setColor(1.0-0.1*(float)color, 0.55, 0.05+0.1*(float)color);
      vtkwin.addToRenderer(transformedModel->getActor());
      // Save in a list in order to delete outside this loop
      transformedModelList.push_back(transformedModel);

    // Cleanup
    mat_dealloc(mat4x4, 4);
    transformer->Delete();
    //color++;
  }

  // Visualize the scene
  VtkPoints scenePoints(scene);
    scenePoints.selfAdjustPointRadius();
    scenePoints.setColor(0.1, 0.5, 1.0);
    vtkwin.addToRenderer(scenePoints.getActor());

  // Visualize the background
  VtkPoints* backgroundPoints = NULL;
  if ( background )
  {
    backgroundPoints = new VtkPoints(background);
    backgroundPoints->selfAdjustPointRadius();
    backgroundPoints->setColor(0.8, 0.8, 0.8);
    vtkwin.addToRenderer(backgroundPoints->getActor());
  }

  // The main vtk loop
  vtkwin.vtkMainLoop();

  // Cleanup
  for ( list<VtkPolyData*>::iterator it = transformedModelList.begin() ; it != transformedModelList.end() ; ++it )
    delete *it;
  if ( backgroundPoints )
    delete backgroundPoints;
}
