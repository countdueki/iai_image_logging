cmake_minimum_required(VERSION 2.8.3)
project(iai_image_logging)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        roscpp
        nodelet
        dynamic_reconfigure
        iai_image_logging_msgs
        image_transport
        mongodb_store
        )

catkin_package(
          CATKIN_DEPENDS  iai_image_logging_msgs nodelet
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_library(image_logger_nodelet src/image_logger.cpp src/preprocessor.cpp
        )


target_link_libraries(image_logger_nodelet ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(image_logger_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()