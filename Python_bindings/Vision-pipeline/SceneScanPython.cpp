#include "SceneScanPython.h"

using namespace visiontransfer;
// Function for filtering nan values from point clouds
void removeInfFromPointCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
  pcl::PointCloud<pcl::PointXYZI>& cloud_out);
void removeInfFromColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,
  pcl::PointCloud<pcl::PointXYZRGB>& cloud_out);

SceneScanProPython::SceneScanProPython(
  bool verbosity, bool hasColorArg, Eigen::Vector3f minBoxValsArg,
  Eigen::Vector3f maxBoxValsArg, double lowerBrightnessArg,
  double upperBrightnessArg, double lengthConstraintArg,
  Eigen::Matrix4f transformPointCloudArg, Eigen::Vector4f leafSizes)
  : verbose(verbosity), hasColor(hasColorArg),
  transformIsometry(Eigen::Isometry3f(transformPointCloudArg)),
  boxFilter(pcl::CropBox<pcl::PointXYZI>()),
  boxFilterRGB(pcl::CropBox<pcl::PointXYZRGB>()),
  intensityFilter(pcl::PassThrough<pcl::PointXYZI>()),
  color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>()),
  lengthConstraint(lengthConstraintArg),
  grabber_(nullptr), recorder_(nullptr) {
  if (verbose) {
    std::cout << "Creating Camera Object, chosen options:\n"
      "\nverbosity: "
      << verbose << "\nColor: " << hasColor << "\nMinimum Cropbox: \n"
      << minBoxValsArg << "\nMaximum Cropbox: \n"
      << maxBoxValsArg
      << "\nLower Brightness 0=black: " << lowerBrightnessArg
      << "\nUpper Brightness: " << upperBrightnessArg
      << "\nMaximum Distance: " << lengthConstraintArg
      << "\nEye2Robot Transformation: \n"
      << transformPointCloudArg
      << "\nCurrently Leaf Size is bugged! Voxelgridfilter is not set."
      << std::endl;
  }

  // Setup Filters
  intensityFilter.setFilterFieldName("intensity");
  setBrightness(lowerBrightnessArg, upperBrightnessArg);
  setRGBFilter();
  setBoxValues(minBoxValsArg, maxBoxValsArg);
  vox.setLeafSize(leafSizes);
  voxrgb.setLeafSize(leafSizes);
}

SceneScanProPython::~SceneScanProPython() {
  if (grabber_) {
    grabber_->stop();
    grabber_.reset(nullptr);
  }
  if (recorder_) {
    recorder_->stop();
    recorder_.reset(nullptr);
  }
}

void SceneScanProPython::setPythonCallback(
  std::function<void(const Eigen::Matrix3Xf&, const Eigen::VectorXf&)>
  pyFcn) {
  if (hasColor) {
    std::cout << "Error: Can't register XYZI Callback for RGB camera."
      << std::endl;
  }
  else {
    pyCallback_ = pyFcn;
  }
}

void SceneScanProPython::setPythonCallbackColor(
  std::function<void(const Eigen::Matrix3Xf&, const Eigen::Matrix3Xi&)>
  pyFcn) {
  if (!hasColor) {
    std::cout << "Error: Can't register XYZRGB Callback for Monochrome camera."
      << std::endl;
  }
  else {
    pyCallbackColor_ = pyFcn;
  }
}

void SceneScanProPython::setPythonCallbackWithTimeStamps(
  std::function<void(const Eigen::Matrix3Xf&, const Eigen::VectorXf&,
    const std::vector<unsigned long long int>&)>
  pyFcn) {
  pyCallbackTimeStamps_ = pyFcn;
}

void SceneScanProPython::start() {
  if (pyCallback_ || pyCallbackColor_) {
    grabber_ = std::make_unique<SceneScanProGrabber>();
    if (hasColor) {
      std::function<void(
        const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)>
        f = [this](const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
          & cloud) {
            thread = std::thread(&SceneScanProPython::filterAndPassPointsRGB,
              this, cloud, timeStampCam);
            thread.detach();
      };
      grabber_->registerCallback(f);
      grabber_->start();
    }
    else {
      std::function<void(
        const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)>
        f = [this](const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr
          & cloud) {
            thread = std::thread(&SceneScanProPython::filterAndPassPoints, this,
              cloud, timeStampCam);
            thread.detach();
      };
      grabber_->registerCallback(f);
      grabber_->start();
    }
  }
  else if (pyCallbackTimeStamps_) {
    grabber_ = std::make_unique<SceneScanProGrabber>();
    if (hasColor) {
      std::function<void(
        const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)>
        f = [this](const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
          & cloud) {
            timeStampCam.clear();
            pointCloudCPP =
              std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch())
              .count();
            timeCam = grabber_->sec * 1e6 + grabber_->microsec;
            timeStampCam.push_back(timeCam);
            timeStampCam.push_back(pointCloudCPP);
            thread = std::thread(&SceneScanProPython::filterAndPassPointsRGB,
              this, cloud, timeStampCam);
            thread.detach();
      };
      grabber_->registerCallback(f);
      grabber_->start();
    }
    else {
      std::function<void(
        const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)>
        f = [this](const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr
          & cloud) {
            timeStampCam.clear();
            pointCloudCPP =
              std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch())
              .count();
            timeCam = grabber_->sec * 1e6 + grabber_->microsec;
            timeStampCam.push_back(timeCam);
            timeStampCam.push_back(pointCloudCPP);
            thread = std::thread(&SceneScanProPython::filterAndPassPoints, this,
              cloud, timeStampCam);
            thread.detach();
      };
      grabber_->registerCallback(f);
      grabber_->start();
    }
  }
  else {
    std::cout << "Failed to start Camera. Register a Callback Function first."
      << std::endl;
  }
}

void SceneScanProPython::stop() {
  grabber_->stop();
  // grabber_.reset(nullptr);
}

void SceneScanProPython::startReplay(std::string folderpath, double cycleTime,
  std::string order) {
  // input parsing so we only need to exchange standard datatypes over pybind
  std::filesystem::path folderToReadFrom(folderpath);
  PcdSequenceGrabber::FileIterationOrder readorder;
  if (order.compare("up")) {
    readorder = PcdSequenceGrabber::FileIterationOrder::ASC;
  }
  else if (order.compare("down")) {
    readorder = PcdSequenceGrabber::FileIterationOrder::DESC;
  }
  else {
    // default is down
    readorder = PcdSequenceGrabber::FileIterationOrder::ASC;
  }
  if (pyCallback_) {
    recorder_ = std::make_unique<PcdSequenceGrabber>(folderToReadFrom,
      cycleTime, readorder);
    std::function<void(
      const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)>
      f = [this](const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr
        & cloud) {
          thread = std::thread(&SceneScanProPython::passPoints, this, cloud);
          thread.detach();
    };

    recorder_->registerCallback(f);
    recorder_->start();
  }
}

void SceneScanProPython::stopReplay() {
  recorder_->stop();
  recorder_.reset(nullptr);
}

void SceneScanProPython::setTransformPointCloud(const Eigen::Matrix4f& newTF) {
  transformIsometry = Eigen::Isometry3f(newTF);
}

Eigen::Matrix4f SceneScanProPython::getTransformPointCloud() {
  return transformIsometry.matrix();
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> SceneScanProPython::getBoxValues() {
  std::pair<Eigen::Vector3f, Eigen::Vector3f> boxFilterLimits;
  boxFilterLimits.first = boxFilter.getMin().head<3>();
  boxFilterLimits.second = boxFilter.getMax().head<3>();
  return boxFilterLimits;
}

void SceneScanProPython::setBoxValues(const Eigen::Vector3f& minVals,
  const Eigen::Vector3f& maxVals) {
  Eigen::Vector4f min = (Eigen::Vector4f() << minVals, 1.0).finished();
  Eigen::Vector4f max = (Eigen::Vector4f() << maxVals, 1.0).finished();
  boxFilter.setMin(min);
  boxFilter.setMax(max);
  boxFilterRGB.setMin(min);
  boxFilterRGB.setMax(max);
}

void SceneScanProPython::setBrightness(double lower, double upper) {
  intensityFilter.setFilterLimits(lower, upper); // [min, max] intensity
}

void SceneScanProPython::setVoxelGridFilter(Eigen::Vector4f leafSizes) {
  vox.setLeafSize(leafSizes);
  voxrgb.setLeafSize(leafSizes);
}

void SceneScanProPython::setRGBFilter(int R_lower, int R_upper, int G_lower,
  int G_upper, int B_lower, int B_upper) {
  _R_lower = R_lower;
  _G_lower = G_lower;
  _B_lower = B_lower;
  _R_upper = R_upper;
  _G_upper = G_upper;
  _B_upper = B_upper;

  color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
      "r", pcl::ComparisonOps::LT, _R_upper)));
  color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
      "r", pcl::ComparisonOps::GT, _R_lower)));
  color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
      "g", pcl::ComparisonOps::LT, _G_upper)));
  color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
      "g", pcl::ComparisonOps::GT, _G_lower)));
  color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
      "b", pcl::ComparisonOps::LT, _B_upper)));
  color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
      "b", pcl::ComparisonOps::GT, _B_lower)));

  // build the filter
  condrem.setCondition(color_cond);
}

void SceneScanProPython::startRecording(std::string path) {
  recording_ = true;
  recordingPath_ = path;
}

void SceneScanProPython::stopRecording() { recording_ = false; }

void SceneScanProPython::filterAndPassPoints(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& inCloud,
  std::vector<unsigned long long int> timeStamps) {
  if (pyCallback_ ||
    pyCallbackTimeStamps_) { // do nothing if no python callback function is
  // registered to process the point
  // clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr removedNaNCloud(
      new pcl::PointCloud<pcl::PointXYZI>());

    // Apply Voxel Grid filter
    // vox.setInputCloud(inCloud);
    // vox.filter(*filteredCloud);

    removeInfFromPointCloud(*inCloud, *removedNaNCloud);

    intensityFilter.setInputCloud(removedNaNCloud);
    intensityFilter.filter(*filteredCloud);

    // This rotates the Pointcloud
    pcl::transformPointCloud(*filteredCloud, *filteredCloud,
      transformIsometry.inverse());

    boxFilter.setInputCloud(filteredCloud);
    boxFilter.filter(*filteredCloud);

    // save pointcloud as PCD file if recording_ is set true
    if (recording_) {
      if (filteredCloud->size() > 0) {
        std::stringstream filePath;
        filePath << recordingPath_ << "pointCloud_"
          << boost::posix_time::to_iso_string(
            boost::posix_time::microsec_clock::local_time())
          << ".pcd";

        pcl::io::savePCDFileASCII(filePath.str(), *filteredCloud);
      }
    }

    const auto len = filteredCloud->size();
    Eigen::Matrix3Xf locations = Eigen::Matrix3Xf(3, (Eigen::Index)len);
    Eigen::VectorXf intensities = Eigen::VectorXf((Eigen::Index)len);

    for (size_t i = 0; i < len; i++) {
      const pcl::PointXYZI p = filteredCloud->at(i);
      locations.col(i) = Eigen::Vector3f(p.x, p.y, p.z);
      intensities(i) = p.intensity;
    }

    if (pyCallback_) {
      pyCallback_(locations, intensities);
    }
    if (pyCallbackTimeStamps_) {
      pointCloudFilteredCPP =
        std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
        .count();
      timeStamps.push_back(pointCloudFilteredCPP);
      pyCallbackTimeStamps_(locations, intensities, timeStamps);
    }
  }
}

void SceneScanProPython::filterAndPassPointsRGB(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inCloud,
  std::vector<unsigned long long int> timeStamps) {
  if (pyCallbackColor_ ||
    pyCallbackColorTimeStamps_) { // do nothing if no python callback function
                                  // is
  // registered to process the point
  // clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removedNaNCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());

    // Apply Voxel Grid filter
    voxrgb.setInputCloud(inCloud);
    voxrgb.filter(*filteredCloud);

    removeInfFromColorCloud(*filteredCloud, *removedNaNCloud);

    if (mtx.try_lock()) {
      // Build the filter
      condrem.setInputCloud(removedNaNCloud);
      condrem.filter(*filteredCloud);

      // This rotates the Pointcloud
      pcl::transformPointCloud(*filteredCloud, *filteredCloud,
        transformIsometry.inverse());
      // pcl::transformPointCloud(inCloud, *filteredCloud,
      // transformIsometry.inverse());

      boxFilterRGB.setInputCloud(filteredCloud);
      boxFilterRGB.filter(*filteredCloud);
      mtx.unlock();
    }
    else {
      return;
    }

    const auto len = filteredCloud->size();
    Eigen::Matrix3Xf locations = Eigen::Matrix3Xf(3, (Eigen::Index)len);
    Eigen::Matrix3Xi colors = Eigen::Matrix3Xi(3, (Eigen::Index)len);
    // colors = Eigen::Matrix3Xf(3, (Eigen::Index)len).cast<int>();
    if (recording_) {
      if (filteredCloud->size() > 0) {
        std::stringstream filePath;
        filePath << recordingPath_ << "pointCloud_"
          << boost::posix_time::to_iso_string(
            boost::posix_time::microsec_clock::local_time())
          << ".pcd";

        pcl::io::savePCDFileASCII(filePath.str(), *filteredCloud);
      }
    }
    for (size_t i = 0; i < len; i++) {
      const pcl::PointXYZRGB p = filteredCloud->at(i);
      locations.col(i) = Eigen::Vector3f(p.x, p.y, p.z);
      colors.col(i) = Eigen::Vector3i(p.r, p.g, p.b);
    }
    if (pyCallback_) {
      pyCallbackColor_(locations, colors);
    }
    if (pyCallbackColorTimeStamps_) {
      pointCloudFilteredCPP =
        std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
        .count();
      timeStamps.push_back(pointCloudFilteredCPP);
      pyCallbackColorTimeStamps_(locations, colors, timeStamps);
    }
  }
}

void SceneScanProPython::passPoints(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& inCloud) {
  // do nothing if no python callback function is
  // registered to process the point
  // clouds
  if (pyCallback_) {
    // filter out nan values
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
    removeInfFromPointCloud(*inCloud, *filteredCloud);

    const auto len = filteredCloud->size();
    Eigen::Matrix3Xf locations = Eigen::Matrix3Xf(3, (Eigen::Index)len);
    Eigen::VectorXf intensities = Eigen::VectorXf((Eigen::Index)len);
    for (size_t i = 0; i < len; i++) {
      const pcl::PointXYZI p = filteredCloud->at(i);
      locations.col(i) = Eigen::Vector3f(p.x, p.y, p.z);
      intensities(i) = p.intensity;
    }
    pyCallback_(locations, intensities);
  }
}

// Function for filtering nan values from recorded point clouds
void removeInfFromPointCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
  pcl::PointCloud<pcl::PointXYZI>& cloud_out) {
  std::vector<int> index;
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.resize(cloud_in.size());
    cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  }
  // Reserve enough space for the indices
  index.resize(cloud_in.size());
  std::size_t j = 0;
  for (std::size_t i = 0; i < cloud_in.size(); ++i) {
    if (!std::isfinite(cloud_in[i].x) || !std::isfinite(cloud_in[i].y) ||
      !std::isfinite(cloud_in[i].z))
      continue;
    cloud_out[j] = cloud_in[i];
    index[j] = i;
    j++;
  }
  if (j != cloud_in.size()) {
    // Resize to the correct size
    cloud_out.resize(j);
    index.resize(j);
  }

  cloud_out.height = 1;
  cloud_out.width = static_cast<std::uint32_t>(j);

  // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
  cloud_out.is_dense = true;
}
void removeInfFromColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,
  pcl::PointCloud<pcl::PointXYZRGB>& cloud_out) {
  std::vector<int> index;
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.resize(cloud_in.size());
    cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  }
  // Reserve enough space for the indices
  index.resize(cloud_in.size());
  std::size_t j = 0;
  for (std::size_t i = 0; i < cloud_in.size(); ++i) {
    if (!std::isfinite(cloud_in[i].x) || !std::isfinite(cloud_in[i].y) ||
      !std::isfinite(cloud_in[i].z))
      continue;
    cloud_out[j] = cloud_in[i];
    index[j] = i;
    j++;
  }
  if (j != cloud_in.size()) {
    // Resize to the correct size
    cloud_out.resize(j);
    index.resize(j);
  }

  cloud_out.height = 1;
  cloud_out.width = static_cast<std::uint32_t>(j);

  // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
  cloud_out.is_dense = true;
}
