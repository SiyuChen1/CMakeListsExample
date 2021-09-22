#pragma once

#include <pcl/filters/extract_indices.h>
#include <pcl/pcl_base.h>

#include <chrono>
#include <mutex>
#include <exception>
#include <functional>
#include <iostream>
#include <stdio.h>
#include <string>
#include <thread>
#include <visiontransfer/deviceenumeration.h>
#include <visiontransfer/imageset.h>
#include <visiontransfer/imagetransfer.h>
#include <visiontransfer/reconstruct3d.h>

#include "scene_scan_grabber.h"
#include "pcd_sequence_grabber.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h> 


// Point cloud transformation
#include <pcl/common/transforms.h>

// crop box filter
#include <pcl/filters/crop_box.h>

// filter by coordinate
#include <pcl/filters/passthrough.h>

// boost
#include <boost/date_time/posix_time/posix_time.hpp>

class SceneScanProPython
{
public:
  SceneScanProPython(bool verbosity = true,
    bool hasColorArg = false,
                     Eigen::Vector3f minBoxValsArg = Eigen::Vector3f(-0.4, -0.57, 0.1),
                     Eigen::Vector3f maxBoxValsArg = Eigen::Vector3f(0.4, -0.2, 0.2),
                     double lowerBrightnessArg = 0, double upperBrightnessArg = 0.2,
                     double lengthConstraintArg = 6, Eigen::Matrix4f transformPointCloudArg =
                    (Eigen::Matrix4f() << 0.00671539085610851, 0.999927645169281, -0.00998037834289894,
                    0.503613864554976, 0.878173962248163, -0.0106708935555742, -0.478222358385812,
                    0.0117510827747765, -0.478294256242966, -0.00555305834142869, -0.878182081340792,
                    1.20128499, 0, 0, 0, 1)
                        .finished(),
                        Eigen::Vector4f leafSizes = (Eigen::Vector4f() << 0.001, 0.001, 0.001, 1).finished());
  /**
  Constructor, optional Arguments:

  \param verbosity: toggle console output On/Off
  \param hasColorArg: define wheter color mode or not
  \param minBoxValsArg: define the minimum Box Values [x y z]
  \param maxBoxValsArg: define the maximum Box Values [x y z]
  \param lowerBrightnessArg: define the Lower Brightness Limit (0 is black)
  \param upperBrightnessArg: define the upper brightness Limit 1 is white
  \param lengthConstraintArg: define the maximum length to be considered
  \param transformPointCloudArg: defines the Transformation from Eye 2 Robot
  \param leafSizes: defines the x y z 1 Voxel Size
  **/

  ~SceneScanProPython();
  // Destructor disconnects the camera


  void setPythonCallback(std::function<void(const Eigen::Matrix3Xf &, const Eigen::VectorXf &)> pyFcn);
  // Register a Python Callback


  void setPythonCallbackColor(std::function<void(const Eigen::Matrix3Xf &, const Eigen::Matrix3Xi &)> pyFcn);
  // Register a Python Callback

  
  void setPythonCallbackWithTimeStamps(
      std::function<void(const Eigen::Matrix3Xf &, const Eigen::VectorXf &,
                         const std::vector<unsigned long long int> &)>
          pyFcn);
  // Register a Python Callback

  
  
  void setPythonCallbackColorTimeStamps_(
      std::function<void(const Eigen::Matrix3Xf &, const Eigen::Matrix3Xi &,
                         const std::vector<unsigned long long int> &)>
          pyFcn);
  // Not Functionable
  

  void start();
  // start streaming from the camera:


  void stop();
  // stop streaming from the camera:


  void startReplay(std::string folderpath, double cycleTime,
                   std::string order);
  /** replay point cloud from recorded pcd files:

  \param folderpath: Path to the Folder to start a replay
  \param cycletime: Updaterate
  \param order: ASC(up) or DESC(down) Order
  */

  void stopReplay();
  // Stops Replaying

  void setTransformPointCloud(const Eigen::Matrix4f &newTF);
  /**
  Set a new Transformation to transform the pointCloud with

  \param newTF: 4x4 Eigen::Matrix
  */
  void setBoxValues(const Eigen::Vector3f &minVals, const Eigen::Vector3f &maxVals);
  /**
  Set the Box which is used as a cutout

  \param minVals: Eigen::Vector3f with [x y z] Minimum of the Box
  \param maxVals: Eigen::Vector3f with [x y z] Maximum of the Box
  */
  std::pair<Eigen::Vector3f, Eigen::Vector3f> getBoxValues();
  /**
  Return the active Box Values
  */
  void setBrightness(double lower, double upper);
  /**
  Set Brightness levels to cut out pointcloud

  \param lower: lower Limit (0 is black)
  \param upper: upper limit (1 is white)
  */

  void setRGBFilter(int R_lower = 0, int R_upper = 255, int G_lower = 0, int G_upper = 255, int B_lower = 0, int B_upper = 255);
  /**
  Set Brightness levels to cut out pointcloud

  \param R_lower: lower Limit (0 is not Red)
  \param R_upper: upper limit (255 is Red)
  \param G_lower: lower Limit (0 is not green)
  \param G_upper: upper limit (255 is green)
  \param B_lower: lower Limit (0 is not blue)
  \param B_upper: upper limit (255 is blue)
  **/

  Eigen::Matrix4f getTransformPointCloud();
  /**
  Returns the current Transformation Matrix Eigen::Matrix4f
  */

  void setVoxelGridFilter(Eigen::Vector4f leafSizes);
  /**
   * @brief Set Leaf Sizes for Voxel Grid
   * 
   * @param leafSizes: x y z and 1
   */

  void startRecording(std::string path);
  /**
  Starts a recording
  \param path: path to folder to store pcd files
  */

  void stopRecording();
  // Stops the Recording

protected:
  bool verbose;
  bool hasColor;

  std::mutex mtx;

  // For RGB Filtering
  int  _R_lower = 0;
  int  _G_lower = 0;
  int  _B_lower = 0;
  int  _R_upper = 255;
  int  _G_upper = 255;
  int  _B_upper = 255;

  // For recording
  std::string recordingPath_;
  bool recording_ = false;

  // Time Stamp of the Camera
  std::vector<unsigned long long int> timeStampCam;

  /// Transformation from Robot to Camera
  Eigen::Isometry3f transformIsometry;

  /// Box filter object, filters points to be within xyz box
  pcl::CropBox<pcl::PointXYZI> boxFilter;
  pcl::CropBox<pcl::PointXYZRGB> boxFilterRGB;

  /// Brightness filter, only keep values in grayscale range, where 0 is black and 1 is white
  pcl::PassThrough<pcl::PointXYZI> intensityFilter;
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond;
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;

  /// Voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> vox;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxrgb;

  /// Get Points up to x meter
  double lengthConstraint;

  /// TimeStamps in microseconds, first one of the camera, then the Point Cloud in C++, Then the
  /// Filtered PointCloud
  unsigned long long int timeCam, pointCloudCPP, pointCloudFilteredCPP;

  /// Devices
  std::unique_ptr<SceneScanProGrabber> grabber_;
  std::unique_ptr<PcdSequenceGrabber> recorder_;
  std::function<void(const Eigen::Matrix3Xf &, const Eigen::VectorXf &)> pyCallback_;
  std::function<void(const Eigen::Matrix3Xf &, const Eigen::Matrix3Xi &)> pyCallbackColor_;
  std::function<void(const Eigen::Matrix3Xf &, const Eigen::VectorXf &,
                     const std::vector<unsigned long long int> &)>
      pyCallbackTimeStamps_;
  std::function<void(const Eigen::Matrix3Xf &, const Eigen::Matrix3Xi &,
                     const std::vector<unsigned long long int> &)>
      pyCallbackColorTimeStamps_;

  void filterAndPassPoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &inCloud, std::vector<unsigned long long int> timeStamps);
  void filterAndPassPointsRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &inCloud, std::vector<unsigned long long int> timeStamps);
  void passPoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &inCloud);

  // Thread for Filtering
  std::thread thread;
};
