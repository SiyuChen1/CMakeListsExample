#pragma once 

#include <pcl/io/pcd_io.h>
#include <pcl/io/grabber.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <optional>
#include <thread>
#include <chrono>

/** Grabs pointcloud data from a list of enumerated PCD files in a folder.
 *
 * Other code connects via registering calback functions. Each point cloud is read and sent to the registered callback.
 * The current file is remembered, so when the reading is stopped in the middle, the next start() will cause to continue, unless
 * resetFilesItr() is called before.
 * If all files are read, the grabber stops by itself.
 *
 * ```{.cpp}
 * // defined in pcl::Grabber
 * grabber.registerCallback(boost::function<void(const typename pcl::PointCloud<PointT>::ConstPtr
 * &f);
 * ```
 *
 * Reading the PCD files is performed on a separate thread.
 * */
class PcdSequenceGrabber : public pcl::Grabber{


public:
  // callback signature typedefs:
  using sig_cb_point_cloud = void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &);
  using sig_cb_point_cloud_i = void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &);
  using sig_cb_point_cloud_rgb = void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);

  /// Order to iterate in the directory. ACS ... A->Z, 0->9, DESC is reverse.
  enum class FileIterationOrder { ASC, DESC};

  //@{
  /** Constructor 
   * \param folderToReadFrom ... path to folder, where the PCD files are located in
   * \param cycleTime ... (>=0.0) time between two pointcloud readings [s]
   * \param order ... FileIterationOrder::ASC or FileIterationOrder::DESC, whether to forward or backard iterate over files
  */
  PcdSequenceGrabber(const std::filesystem::path& folderToReadFrom, double cycleTime = 0.0, FileIterationOrder order = FileIterationOrder::ASC);

  PcdSequenceGrabber(PcdSequenceGrabber &other) = delete;
  //@}

  PcdSequenceGrabber operator=(PcdSequenceGrabber &other) = delete;

  ~PcdSequenceGrabber();

  /** start the grabbing, if at least one callback is registered.
   *
   * The grabbing is performed until stop() is called.
   * */
  void start() override;

  /** stop the grabbing until start() is called.
   * */
  void stop() override;

  /** returns the grabber's name
   * */
  std::string getName() const override;

  /** Indicates, whether the grabber is currently running */
  bool isRunning() const override;

  /** Returns the fps number (estimated).
   */
  float getFramesPerSecond() const override;

  /** Reset the files iteration to start from begin on next call to start().
   * */
  inline void resetFilesItr(){ filesItr.reset(); }

protected:
  /** Runs the grabber by starting a loop and grabbing the pointclouds.
   *
   * Is executed by start() on a separate thread.
   * */
  void run();

  // signals:
  /// signal for `pcl::PointXYZ` pointcloud
  boost::signals2::signal<sig_cb_point_cloud> *point_cloud_signal_;

  /// signal for `pcl::PointXYZI` pointcloud
  boost::signals2::signal<sig_cb_point_cloud_i> *point_cloud_i_signal_;

  /// signal for `pcl::PointXYZRGB` pointcloud
  boost::signals2::signal<sig_cb_point_cloud_rgb> *point_cloud_rgb_signal_;

  //@{
  /// flags, whether XYZ and/or  XYZRGB subscribers exist. Computed and stored at `start()`.
  bool need_xyz_, need_xyzi_, need_xyzrgb_;
  //@}

  // flag if thread is currently running
  bool is_running_;

  /// storage for pointcloud xyz
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud;

  /// storage for pointcloud xyz + intensity (geyscale)
  pcl::PointCloud<pcl::PointXYZI>::Ptr xyziCloud;

  /// storage for pointcloud xyz + rgb (needs color camera)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgbCloud;

  /// sequence number counter for enumerating pointclouds
  uint32_t sequenceNo_;

  /// the thread storage, where the run() function is spawned
  std::thread thread_;

  /// path to folder to read from:
  std::filesystem::path folderToRead;

  /// order to read the PCD files.
  FileIterationOrder order;

  /// next file to process:
  std::vector<std::filesystem::directory_entry> pcdFiles;

  /// Current position in files:
  std::optional<std::vector<std::filesystem::directory_entry>::iterator> filesItr;

  /// cycle time [s] between two pointCloud readings.
  std::chrono::duration<double> cycleTime;
};