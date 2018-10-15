cmake_minimum_required(VERSION 2.8.3)
project(iai_image_logging)

add_compile_options(-std=c++11)

# OpenCV needs to be packaged seperately
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED
        roscpp
        cv_bridge
        image_geometry
        dynamic_reconfigure
        iai_image_logging_msgs
        mongodb_store
        )

catkin_package(CATKIN_DEPENDS
        iai_image_logging_msgs
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )
add_library(storage_sub
        src/storage_sub.cpp
        src/blur_detector.h )

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
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        storage_sub

        )

# -- INSTALL -- #

# add python scripts
install(PROGRAMS
        # scripts/config_client.py
        scripts/add.py
        scripts/add_yaml.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
