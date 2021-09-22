#pragma once

#include <pcl/common/io.h>
#include <pcl/io/grabber.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include<boost/make_shared.hpp>

#include <visiontransfer/deviceenumeration.h>
#include <visiontransfer/imageset.h>
#include <visiontransfer/imagetransfer.h>
#include <visiontransfer/reconstruct3d.h>

#include <exception>
#include <thread>

/** Grabs pointcloud data from Nerian Scene Scan Pro cameras.
 *
 * Other code connects via registering calback functions. These are called each time a new image is
 * available.
 *
 * ```{.cpp}
 * // defined in pcl::Grabber
 * grabber.registerCallback(boost::function<void(const typename pcl::PointCloud<PointT>::ConstPtr
 * &f);
 * ```
 *
 * The connection with the camera is running on a separate thread.
 * */
class SceneScanProGrabber : public pcl::Grabber
{
public:
    // callback signature typedefs:
    using sig_cb_nerian_point_cloud = void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &);
    using sig_cb_nerian_point_cloud_i = void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &);
    using sig_cb_nerian_point_cloud_rgb = void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);

    //@{
    /** Constructor */
    SceneScanProGrabber();

    SceneScanProGrabber(SceneScanProGrabber &other) = delete;
    //@}

    SceneScanProGrabber operator=(SceneScanProGrabber &other) = delete;

    ~SceneScanProGrabber();

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

    /** Returns the fps number from the `libvisiontransfer` interface.
   */
    float getFramesPerSecond() const override;

    // Trigger reception of the timestamp
    bool need_timestamp_ = true;

    // Timestamp of the camera
    int sec, microsec;

protected:
    /**Runs the grabber by starting a loop and grabbing the pointclouds.
   *
   * Is executed by start() on a separate thread.
   * */
    void run();

    /** initializes the storage for pointclouds once.
   *
   * This is executed once, after the start() function is called.
   *
   * \param
   * */
    void initializePointClouds(const visiontransfer::ImageSet &p);

    /** Sets the header of a plc::PointCloud<PointT> to the values retreived by an ImageSet.
   *
   * \tparam PointT the pcl::Point... type
   * \param cloud The PointCloud to set the header for
   * \param p The ImageSet object to extract the meta data from
   * \param frame The name of the camera frame
   *
   * */
    template <typename PointT>
    void setHeader(const typename pcl::PointCloud<PointT>::Ptr cloud,
                   const visiontransfer::ImageSet &p, const char *frameId = "Cam")
    {
        if (cloud)
        {
            // int sec, microsec;
            // p.getTimestamp(sec, microsec);

            cloud->header.frame_id = frameId;
            cloud->header.seq = p.getSequenceNumber();
            cloud->header.stamp = sec * 1000000LL + microsec;
            cloud->width = p.getWidth();
            cloud->height = p.getHeight();
            cloud->is_dense = true;
        }
    }

    /** Set new XYZI data to internal xyziCloud.
   *
   * \param pointMap pointer to current point map (received from Reconstruct3d object)
   * \param p ImageSet, only accessed for metadata
   *
   * \note The code is mainly from `reconstruct3d-pcl.h` from Nerian's libvisiontransfer. It is only
   * here to avoid allocating a new PointCloud object in each time step.
   */
    void setXYZI(float *pointMap, const visiontransfer::ImageSet &p);

    /** Set new XYZRGB data to internal xyzrgbCloud.
   *
   * \param pointMap pointer to current point map (received from Reconstruct3d object)
   * \param p ImageSet, only accessed for metadata
   *
   * \note The code is mainly from `reconstruct3d-pcl.h` from Nerian's libvisiontransfer. It is only
   * here to avoid allocating a new PointCloud object in each time step.
   */
    void setXYZRGB(float *pointMap, const visiontransfer::ImageSet &p);

    // signals:
    /// signal for `pcl::PointXYZ` pointcloud
    boost::signals2::signal<sig_cb_nerian_point_cloud> *point_cloud_signal_;

    /// signal for `pcl::PointXYZI` pointcloud
    boost::signals2::signal<sig_cb_nerian_point_cloud_i> *point_cloud_i_signal_;

    /// signal for `pcl::PointXYZRGB` pointcloud
    boost::signals2::signal<sig_cb_nerian_point_cloud_rgb> *point_cloud_rgb_signal_;

    //@{
    /// flags, whether XYZ and/or  XYZRGB subscribers exist. Computed and stored at `start()`.
    bool need_xyz_, need_xyzi_, need_xyzrgb_;
    //@}

    // flag if thread is currently running
    bool is_running_;

    /// connection object to libvisiontransfer for unique camera identification
    visiontransfer::DeviceInfo deviceInfo_;

    /// Connection object for retreiving data
    std::unique_ptr<visiontransfer::ImageTransfer> device_;

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
};