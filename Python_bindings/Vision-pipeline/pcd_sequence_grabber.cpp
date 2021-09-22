#include <pcd_sequence_grabber.h>

PcdSequenceGrabber::PcdSequenceGrabber(const std::filesystem::path &folderToReadFrom, double cycleTime,
                                       FileIterationOrder order)
    : pcl::Grabber(), point_cloud_signal_(createSignal<sig_cb_point_cloud>()),
      point_cloud_i_signal_(createSignal<sig_cb_point_cloud_i>()),
      point_cloud_rgb_signal_(createSignal<sig_cb_point_cloud_rgb>()),
      is_running_(false),
      xyzCloud(new pcl::PointCloud<pcl::PointXYZ>()),
      xyziCloud(new pcl::PointCloud<pcl::PointXYZI>()),
      xyzrgbCloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
      folderToRead(folderToReadFrom),
      order(order),
      cycleTime(std::chrono::duration<double>(std::max(cycleTime, 0.0))){
  if (!std::filesystem::is_directory(folderToRead) || std::filesystem::is_empty(folderToRead)) {
    PCL_ERROR("Path %s is invalid. Please check the path and whether it contains PCD files.\n", folderToRead.c_str());
    folderToRead.clear();
  }
}

PcdSequenceGrabber::~PcdSequenceGrabber() {
  if (is_running_) {
    stop();
  }

  disconnect_all_slots<sig_cb_point_cloud>();
  disconnect_all_slots<sig_cb_point_cloud_i>();
  disconnect_all_slots<sig_cb_point_cloud_rgb>();
}

void PcdSequenceGrabber::start() {
  if (!is_running_ && !folderToRead.empty()) {
    need_xyz_ = (num_slots<sig_cb_point_cloud>() > 0);
    need_xyzi_ = (num_slots<sig_cb_point_cloud_i>() > 0);
    need_xyzrgb_ = (num_slots<sig_cb_point_cloud_rgb>() > 0);

    if (need_xyz_ || need_xyzi_ || need_xyzrgb_) {
      // set running and spawn thread:
      is_running_ = true;

      thread_ = std::thread(&PcdSequenceGrabber::run, this);
    }
  }
  else {
    // std::cout<<is_running_<<std::endl;
    // std::cout<<folderToRead.empty()<<std::endl;
    PCL_ERROR("Unable to start PCD sequence reader.");
  }
  /// TODO(hze): log, if already running, or path invalid
}

void PcdSequenceGrabber::stop() {
  if (is_running_) {
    is_running_ = false;
    thread_.join();
  }
}

std::string PcdSequenceGrabber::getName() const {
  return std::string("Folder Grabber: ") + folderToRead.string();
}

bool PcdSequenceGrabber::isRunning() const { return is_running_; }

float PcdSequenceGrabber::getFramesPerSecond() const {
  return 0.0; // TODO(hze): How to pull this off?
}

void PcdSequenceGrabber::run() {
  if (!filesItr.has_value()) { // means: running through files was not started already
    // discover directory contents:
    for (const auto &dirEntry : std::filesystem::directory_iterator(folderToRead)) {
      if (dirEntry.is_regular_file() && dirEntry.path().extension() == ".pcd") {
        pcdFiles.push_back(dirEntry);
      }
    }
    std::sort(pcdFiles.begin(), pcdFiles.end());

    filesItr = pcdFiles.begin();
  }

  std::chrono::high_resolution_clock::time_point lastTime = std::chrono::high_resolution_clock::now();

  // iterate over them:
  while (is_running_) {
    // read pcd file:
    if (need_xyz_) {
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(filesItr.value()->path().string(), *xyzCloud) == -1) {
        PCL_ERROR("Unable to load XYZ PointCloud from %s \n", filesItr.value()->path().c_str());
      } else {
        point_cloud_signal_->operator()(xyzCloud);
      }
    }

    if (need_xyzi_) {
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(filesItr.value()->path().string(), *xyziCloud) ==
          -1) {
        PCL_ERROR("Unable to load XYZI PointCloud from %s \n", filesItr.value()->path().c_str());
      } else {
        point_cloud_i_signal_->operator()(xyziCloud);
      }
    }

    if (need_xyzrgb_) {
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filesItr.value()->path().string(), *xyzrgbCloud) ==
          -1) {
        PCL_ERROR("Unable to load XYZRGB PointCloud from %s \n", filesItr.value()->path().c_str());
      } else {
        point_cloud_rgb_signal_->operator()(xyzrgbCloud);
      }
    }

    // then iterate or end.
    filesItr.value()++;
    if (filesItr.value() == pcdFiles.end()) {
      is_running_ = false;
      resetFilesItr();
    }
    else{
      std::this_thread::sleep_until(lastTime + cycleTime);

      lastTime = std::chrono::high_resolution_clock::now();
    }
    
  }
}