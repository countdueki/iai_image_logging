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
add_library(iai_subscriber
        src/iai_subscriber.cpp
        include/header/iai_subscriber.h
        include/features/blur_detector.h include/features/similarity_detector.h include/features/motion_detector.h)

add_executable(iai_updater
        src/iai_updater.cpp
        include/header/iai_updater.h
        include/header/iai_subscriber.h)

target_link_libraries(iai_updater
        ${catkin_LIBRARIES}
        )

add_executable(iai_configurator
        src/iai_configurator.cpp
        include/header/iai_configurator.h


        )
target_link_libraries(iai_configurator
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        iai_subscriber

        )

# -- INSTALL -- #

# add python scripts
install(PROGRAMS
        # scripts/config_client.py
        scripts/insert.py
        scripts/remove.py
        scripts/behave.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
