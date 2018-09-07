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
        LIBRARIES ${PROJECT_NAME}

        CATKIN_DEPENDS  iai_image_logging_msgs nodelet
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_library(iai_image_logging src/image_logger.cpp src/preprocessor.cpp
        )


target_link_libraries(iai_image_logging ${catkin_LIBRARIES})


#add_library(initiatior src/Initiator.cpp)
#
#target_link_libraries(initiator ${catkin_LIBRARIES})
#
#
#add_executable(initiator
#        src/Initiator.cpp)
#
if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(iai_image_logging ${catkin_EXPORTED_LIBRARIES})
endif()