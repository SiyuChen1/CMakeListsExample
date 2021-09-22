#include "scene_scan_grabber.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

using namespace std::chrono_literals;

SceneScanProGrabber::SceneScanProGrabber()
    : pcl::Grabber(),
      point_cloud_signal_(createSignal<sig_cb_nerian_point_cloud>()),
      point_cloud_i_signal_(createSignal<sig_cb_nerian_point_cloud_i>()),
      point_cloud_rgb_signal_(createSignal<sig_cb_nerian_point_cloud_rgb>()),
      need_xyz_(false), need_xyzi_(false), need_xyzrgb_(false),
      is_running_(false), deviceInfo_(), device_(), sequenceNo_(0), thread_()
{

  // connect with camera:
  // TODO(hze): to support multiple devices, move this outside and accept
  // DeviceInfo object as input here.
  visiontransfer::DeviceEnumeration deviceEnum;
  std::vector<visiontransfer::DeviceInfo> sceneScanDevices =
      deviceEnum.discoverDevices();
  if (sceneScanDevices.size() == 0)
  {
    throw std::underflow_error(
        "No camera found in network. Make sure it is connected.");
  }
  else if (sceneScanDevices.size() > 1)
  {
    throw std::overflow_error("More than one camera device found. We currently "
                              "only support one camera.");
  }

  deviceInfo_ = sceneScanDevices[0];
}

SceneScanProGrabber::~SceneScanProGrabber()
{
  if (is_running_)
  {
    stop();
  }

  disconnect_all_slots<sig_cb_nerian_point_cloud>();
  disconnect_all_slots<sig_cb_nerian_point_cloud_i>();
  disconnect_all_slots<sig_cb_nerian_point_cloud_rgb>();
}

void SceneScanProGrabber::start()
{
  if (!is_running_)
  {
    need_xyz_ = (num_slots<sig_cb_nerian_point_cloud>() > 0);
    need_xyzi_ = (num_slots<sig_cb_nerian_point_cloud_i>() > 0);
    need_xyzrgb_ = (num_slots<sig_cb_nerian_point_cloud_rgb>() > 0);

    if (need_xyz_ || need_xyzi_ || need_xyzrgb_)
    {
      // connect
      if (!deviceInfo_.isCompatible() &&
          (deviceInfo_.getModel() == visiontransfer::DeviceInfo::DeviceModel::SCENESCAN_PRO ||
           deviceInfo_.getModel() == visiontransfer::DeviceInfo::DeviceModel::SCARLET))
      {
        throw std::runtime_error("Incompatible device.");
      }
      device_ = std::make_unique<visiontransfer::ImageTransfer>(deviceInfo_);

      // configure device

      // set running and spawn thread:
      is_running_ = true;

      thread_ = std::thread(&SceneScanProGrabber::run, this);
    }
  }
}

void SceneScanProGrabber::stop()
{
  if (is_running_)
  {
    is_running_ = false;

    thread_.join();
    device_->disconnect();
    // device_ = nullptr;
  }
}

std::string SceneScanProGrabber::getName() const
{
  return "Nerian Scene Scan Pro";
}

bool SceneScanProGrabber::isRunning() const { return is_running_; }

float SceneScanProGrabber::getFramesPerSecond() const
{
  return (float)deviceInfo_.getStatus().getLastFps();
}

void SceneScanProGrabber::run()
{
  visiontransfer::ImageSet p;
  visiontransfer::Reconstruct3D recon3d;

  while (!device_->receiveImageSet(p))
  {
    std::this_thread::sleep_for(5ms);
  }

  initializePointClouds(p);

  while (is_running_)
  {
    if (!device_->isConnected())
    //|| !device_->isConnected())
    {
      stop(); // clean up
      throw std::runtime_error("SceneScan device not connected.");
    }

    if (!need_xyz_ && !need_xyzi_ && !need_xyzrgb_)
    {
      stop(); // clean up
      throw std::logic_error("No data requested. Stop.");
    }

    while (!device_->receiveImageSet(p))
    {
      std::this_thread::sleep_for(5ms);
    }

    float *pointMap = recon3d.createPointMap(p, 0);

    if (need_timestamp_)
    {
      p.getTimestamp(sec, microsec);
      // std::cout << "Picture Taken at sec: " << sec << " microsec: " <<
      // microsec << std::endl;
    }

    if (need_xyz_)
    {
      setHeader<pcl::PointXYZ>(xyzCloud, p, "SceneScanPro");
      memcpy(&xyzCloud->points[0].x, pointMap,
             xyzCloud->width * xyzCloud->height * sizeof(float) * 4);
      point_cloud_signal_->operator()(xyzCloud);
    }

    if (need_xyzi_)
    {
      setHeader<pcl::PointXYZI>(xyziCloud, p, "SceneScanPro");
      setXYZI(pointMap, p);
      point_cloud_i_signal_->operator()(xyziCloud);
    }

    if (need_xyzrgb_)
    {
      setHeader<pcl::PointXYZRGB>(xyzrgbCloud, p, "SceneScanPro");
      setXYZRGB(pointMap, p);
      point_cloud_rgb_signal_->operator()(xyzrgbCloud);
    }
  }
}

void SceneScanProGrabber::initializePointClouds(
    const visiontransfer::ImageSet &p)
{
  xyzCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(p.getWidth(),
                                                              p.getHeight());
  xyziCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
      p.getWidth(), p.getHeight());
  xyzrgbCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(
      p.getWidth(), p.getHeight());
}

void SceneScanProGrabber::setXYZI(float *pointMap,
                                  const visiontransfer::ImageSet &p)
{
  float *pointMapItr = pointMap;
  pcl::PointXYZI *dstPtr = &xyziCloud->points[0];

  if (p.getPixelFormat(0) == visiontransfer::ImageSet::FORMAT_8_BIT_MONO)
  {
    for (int y = 0; y < p.getHeight(); y++)
    {
      unsigned char *rowPtr = p.getPixelData(0) + y * p.getRowStride(0);
      unsigned char *endPtr = rowPtr + p.getWidth();
      for (; rowPtr < endPtr; rowPtr++)
      {
        dstPtr->intensity = static_cast<float>(*rowPtr) / 255.0F;
        dstPtr->x = *pointMapItr++;
        dstPtr->y = *pointMapItr++;
        dstPtr->z = *pointMapItr++;

        pointMapItr++;
        dstPtr++;
      }
    }
  }
  else if (p.getPixelFormat(0) ==
           visiontransfer::ImageSet::FORMAT_12_BIT_MONO)
  {
    for (int y = 0; y < p.getHeight(); y++)
    {
      unsigned short *rowPtr = reinterpret_cast<unsigned short *>(
          p.getPixelData(0) + y * p.getRowStride(0));
      unsigned short *endPtr = rowPtr + p.getWidth();
      for (; rowPtr < endPtr; rowPtr++)
      {
        dstPtr->intensity = static_cast<float>(*rowPtr) / 4095.0F;
        dstPtr->x = *pointMapItr++;
        dstPtr->y = *pointMapItr++;
        dstPtr->z = *pointMapItr++;

        pointMapItr++;
        dstPtr++;
      }
    }
  }
  else
  {
    throw std::runtime_error(
        "Left image does not have a valid greyscale format");
  }
}

void SceneScanProGrabber::setXYZRGB(float *pointMap,
                                    const visiontransfer::ImageSet &p)
{
  float *pointMapItr = pointMap;
  pcl::PointXYZRGB *dstPtr = &xyzrgbCloud->points[0];

  if (p.getPixelFormat(0) != visiontransfer::ImageSet::FORMAT_8_BIT_RGB)
  {
    throw std::runtime_error("Left image is not an RGB image");
  }

  for (int y = 0; y < p.getHeight(); y++)
  {
    unsigned char *rowPtr = p.getPixelData(0) + y * p.getRowStride(0);
    unsigned char *endPtr = rowPtr + 3 * p.getWidth();
    for (; rowPtr < endPtr; rowPtr += 3)
    {
      dstPtr->r = rowPtr[0];
      dstPtr->g = rowPtr[1];
      dstPtr->b = rowPtr[2];
      dstPtr->x = *pointMapItr++;
      dstPtr->y = *pointMapItr++;
      dstPtr->z = *pointMapItr++;

      pointMapItr++;
      dstPtr++;
    }
  }
}
