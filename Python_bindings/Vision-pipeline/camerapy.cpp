#include "SceneScanPython.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

/*
Bindings Class for scene scan Grabber
*/

namespace py = pybind11;

PYBIND11_MODULE(camerapy, m) {
  m.doc() = "Python Bindings to connect to Nerian SceneScan Pro";

  py::class_<SceneScanProPython>(
      m, "SceneScanProPython",
      "Bindings for Nerian SceneScan Pro, initialize with:\n"
      "verbosity = True\n"
      "Color = False\n"
      "MinBoxVals: [-0.4, -0.57, 0.1]\n"
      "MaxBoxVals = [0.4, -0.2, 0.2]\n"
      "LowerBrightness = 0\n"
      "UpperBrightness = 0.2\n"
      "MaxLength = 6\n"
      "TransformEye2Robot = \n[[0.00671539085610851, 0.999927645169281, "
      "-0.00998037834289894, 0.503613864554976],\n"
      " [ 0.878173962248163,-0.0106708935555742, -0.478222358385812, "
      "0.0117510827747765], \n"
      "[    -0.478294256242966, -0.00555305834142869, -0.878182081340792, "
      "1.20128499], \n"
      "[ 0, 0, 0, 1]]\n"
      "LeafSizes = [0.001, 0.001, 0.001, 1]")
      .def(py::init<bool, bool, Eigen::Vector3f, Eigen::Vector3f, double,
                    double, double, Eigen::Matrix4f, Eigen::Vector4f>(),
           py::arg("verbosity") = true, py::arg("Color") = false,
           py::arg("MinBoxVals") = Eigen::Vector3f(-0.4, -0.57, 0.1),
           py::arg("MaxBoxVals") = Eigen::Vector3f(0.4, -0.2, 0.2),
           py::arg("LowerBrightness") = 0, py::arg("UpperBrightness") = 0.2,
           py::arg("MaxLength") = 6,
           py::arg("TransformEye2Robot") =
               (Eigen::Matrix4f() << 0.00671539085610851, 0.999927645169281,
                -0.00998037834289894, 0.503613864554976, 0.878173962248163,
                -0.0106708935555742, -0.478222358385812, 0.0117510827747765,
                -0.478294256242966, -0.00555305834142869, -0.878182081340792,
                1.20128499, 0, 0, 0, 1)
                   .finished(),
           py::arg("LeafSizes") =
               (Eigen::Vector4f() << 0.001, 0.001, 0.001, 1).finished())
      .def("start", &SceneScanProPython::start, "Start the Camera Stream")
      .def("stop", &SceneScanProPython::stop, "Stop the Camera Steam")
      .def("setPythonCallback", &SceneScanProPython::setPythonCallback,
           "Register a Callback which will be called by the Camera after "
           "registering a PointCloud. Note: No Color Callback, Callback fills "
           "PointCloud Matrix and Intensity Vector",
           py::arg("pyFcn"))
      .def("setPythonCallbackColor",
           &SceneScanProPython::setPythonCallbackColor,
           "Register a Callback which will be called by the Camera after "
           "registering a PointCloud. Note: Color Callback, Callback fills "
           "PointCloud Matrix and RGB Matrix",
           py::arg("pyFcn"))
      .def("setPythonCallbackWithTimeStamps",
           &SceneScanProPython::setPythonCallbackWithTimeStamps,
           "Register a Callback which will be called by the Camera after "
           "registering a PointCloud. Note: No Color Callback, Callback fills "
           "PointCloud Matrix, Intensity Vector, timeStamps Vector",
           py::arg("pyFcn"))
      .def("setTransformPointCloud",
           &SceneScanProPython::setTransformPointCloud,
           "Sets the Transformation Eye2Robot as 4x4 Matrix", py::arg("newTF"))
      .def("getTransformPointCloud",
           &SceneScanProPython::getTransformPointCloud,
           "Returns the current Transformation Eye2Robot as a 4x4 Matrix")
      .def("setVoxelGridFilter", &SceneScanProPython::setVoxelGridFilter,
           "Set leaf size np.array(4,) = x y z 1 for the voxel Grid Filter",
           py::arg("leafSizes"))
      .def("setBoxValues", &SceneScanProPython::setBoxValues,
           "Setting the Box Values x y z used for filtering",
           py::arg("minBoxValues"), py::arg("maxBoxValues"))
      .def("getBoxValues", &SceneScanProPython::getBoxValues,
           "Returns the current Boxfilter configuration")
      .def("setBrightness", &SceneScanProPython::setBrightness,
           "Set the Brightness Filter with lower Threshold and upper "
           "Threshold, 0 is black.",
           py::arg("lower"), py::arg("upper"))
      .def("setRGBFilter", &SceneScanProPython::setRGBFilter,
           "Set the RGB Filter Configuration with Lower and Upper Values, 0 - "
           "255.",
           py::arg("R_lower") = 0, py::arg("R_upper") = 255,
           py::arg("G_lower") = 0, py::arg("G_upper") = 255,
           py::arg("B_lower") = 0, py::arg("B_upper") = 255)
      .def("startRecording", &SceneScanProPython::startRecording,
           "Start to save pcd Files from the Camera", py::arg("path"))
      .def("stopRecording", &SceneScanProPython::stopRecording)
      .def("startReplay", &SceneScanProPython::startReplay,
           "Starting to replay point clouds from recorded pcd files",
           py::arg("folderPath"), py::arg("cycleTime"), py::arg("order"))
      .def("stopReplay", &SceneScanProPython::stopReplay,
           "Stops replaying point clouds");
}
