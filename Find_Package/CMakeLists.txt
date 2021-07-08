find_package(PCL EXACT REQUIRED PATHS /usr/local/include/pcl-1.11)
# find_package(PCL 1.10 EXACT REQUIRED PATHS /usr/include/pcl-1.10) 

if(PCL_FOUND)
  message("PCL version is ${PCL_VERSION}")
  message("PCL is found in ${PCL_INCLUDE_DIRS}")
  include_directories( ${PCL_INCLUDE_DIRS} )
endif()