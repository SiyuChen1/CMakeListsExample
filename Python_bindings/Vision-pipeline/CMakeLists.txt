CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

project(Vision-Pipeline
        VERSION 1.1
        LANGUAGES CXX)
if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.4)
    message(FATAL_ERROR "You are on an too old version of GCC. Please update your compiler to at least GCC 8, preferably latest")
endif()    
message("----------------------------------------------------------")
message(" ${PROJECT_NAME} ${${PROJECT_NAME}_VERSION}               ")
message(" Vision-Pipeline Bindings ")
message("")
message("Build and run it with: make")
message("----------------------------------------------------------")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_policy(SET CMP0069 NEW)

file(GLOB_RECURSE camera_py_cpps "*.cpp")
file(GLOB_RECURSE camera_py_hs "*.h")

find_package(PCL QUIET REQUIRED COMPONENTS common io visualization filters features)
MESSAGE( " *** PCL_INCLUDE_DIRS : " ${PCL_INCLUDE_DIRS} )
MESSAGE( " *** PCL_LIB_DIRS : " ${PCL_LIBRARY_DIRS} )
MESSAGE( " *** PCL_LIBRARIES : " ${PCL_LIBRARIES} )
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(LIBDLOVISION_SRC    scene_scan_grabber.cpp
                        SceneScanPython.cpp
                        pcd_sequence_grabber.cpp)

add_library(${PROJECT_NAME} ${LIBDLOVISION_SRC})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
                        
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(PythonLibs 3)

include_directories(${PYTHON_INCLUDE_DIRS})

find_package(PythonInterp REQUIRED)
if(NOT PythonInterp_FOUND)
  message(WARNING "BUILD_PYTHON_BINDINGS is ON, but failed to find PythonInterp. "
    "Disabling camerapy."
  )
  return()
endif()

find_package(pybind11 REQUIRED)
if(NOT pybind11_FOUND)
  message(WARNING "BUILD_PYTHON_BINDINGS is ON, but failed to find pybind11. "
    "Disabling camerapy."
  )
  return()
endif()

# set(Boost_INCLUDE_DIR /usr/local/src/boost_1_46_1)
include_directories(${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR} ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS} ${visiontransfer_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES} visiontransfer stdc++fs)


# Python binding module name

add_library(camerapy SHARED ${camera_py_cpps} ${camera_py_hs})

target_link_libraries(camerapy PRIVATE
    ${PROJECT_NAME}
    ${PYTHON_LIBRARIES} 
    ${Boost_LIBRARIES}
    libpcl_common.so
    libpcl_io.so
    libpcl_visualization.so
    libpcl_filters.so
    libpcl_features.so
)

SET_TARGET_PROPERTIES(camerapy PROPERTIES PREFIX "")

# copy spr_py.so into the folder libs
add_custom_command(TARGET camerapy POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_BINARY_DIR}/src/c++/Vision-pipeline/camerapy.so ${CMAKE_BINARY_DIR}/../libs/)