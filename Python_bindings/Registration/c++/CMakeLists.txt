project(spr
        VERSION 0.1
        LANGUAGES CXX)

message("----------------------------------------------------------")
message(" lib${PROJECT_NAME} ${${PROJECT_NAME}_VERSION}               ")
message(" build with: make")
message("----------------------------------------------------------")

find_package(PCL QUIET REQUIRED COMPONENTS common)

if(PCL_FOUND)
  message("PCL version is ${PCL_VERSION}")
  message("PCL is found in ${PCL_INCLUDE_DIRS}")
  include_directories( ${PCL_INCLUDE_DIRS} )
endif()

find_package(Eigen3 REQUIRED)
message("Eigen3 is found in ${EIGEN3_INCLUDE_DIR}")

set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} src/spr.cpp src/mlle.cpp src/gmm.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} libpcl_io.so)

# add subdirectory:
add_subdirectory(sprpy)