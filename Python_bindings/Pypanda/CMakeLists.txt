project(pypanda
        VERSION 1
        LANGUAGES CXX)

message("----------------------------------------------------------")
message(" ${PROJECT_NAME} ${${PROJECT_NAME}_VERSION}               ")
message(" Panda franka Bindings ")
message("")
message("Build and run it with: make")
message("----------------------------------------------------------")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_policy(SET CMP0069 NEW)

find_package(Franka REQUIRED)

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

add_library(pypanda SHARED pypanda.cpp)

target_link_libraries(pypanda ${PYTHON_LIBRARIES} Franka::Franka)

SET_TARGET_PROPERTIES(pypanda PROPERTIES PREFIX "")

# copy simpleObserver.so into the folder libs
add_custom_command(TARGET pypanda POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                    ${CMAKE_BINARY_DIR}/src/c++/Pypanda/pypanda.so ${CMAKE_BINARY_DIR}/../libs/)