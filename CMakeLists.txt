cmake_minimum_required(VERSION 2.8.3)
project(iai_image_logging)

add_compile_options(-std=c++11)

# OpenCV needs to be packaged seperately
find_package(OpenCV 2 REQUIRED)
find_package(catkin REQUIRED
        nodelet
        roscpp
        image_geometry
        cv_bridge
        dynamic_reconfigure
        iai_image_logging_msgs
        image_transport
        compressed_image_transport
        mongodb_store
        )

catkin_package(CATKIN_DEPENDS
        nodelet
        iai_image_logging_msgs
        )

include_directories(${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})

add_library(iai_nodelets src/iai_nodelet.cpp src/raw_nodelet.cpp src/compressed_nodelet.cpp src/theora_nodelet.cpp)
target_link_libraries(iai_nodelets ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(iai_nodelets ${catkin_EXPORTED_LIBRARIES})
endif()
add_executable(image_logger

        src/image_logger.cpp
        )
target_link_libraries(image_logger
        ${catkin_LIBRARIES}
        )

add_executable(storage
        src/storage.cpp
        )
target_link_libraries(storage
        iai_nodelets
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        )

# -- INSTALL -- #

# add python scripts
install(PROGRAMS
        scripts/config_client.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
