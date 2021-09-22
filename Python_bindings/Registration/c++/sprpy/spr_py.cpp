#include "spr_py.h"

/*
Bindings Class for SPR algorithm
*/

PYBIND11_MODULE(spr_py, m) { 
    //   m.doc() = "This is a Python binding of C++ Library SPR";

  m.def("read_pointcloud_from_pcd",&read_pointcloud_from_pcd);

  py::class_<SPRPython>(m, "SPRPython")
      .def(py::init<const std::map<std::string, float> >())
      .def(py::init<>())
      .def("load_joint_location_from_csv", &SPRPython::load_joint_location_from_csv)
      .def("load_joint_location_from_np", &SPRPython::load_joint_location_from_np)
      .def("load_point_cloud_from_pcd", &SPRPython::load_point_cloud_from_pcd)
      .def("load_point_cloud_from_np", &SPRPython::load_point_cloud_from_np)
      .def("set_verbose", &SPRPython::set_verbose)
      .def("save_registered_to_csv", &SPRPython::save_registered_to_csv)
      .def("get_joint_location", &SPRPython::get_joint_location)
      .def("compute_registered", &SPRPython::compute_registered)
      .def("get_registered_joint_location", &SPRPython::get_registered_joint_location);
}
