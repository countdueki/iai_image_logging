cmake_minimum_required(VERSION 2.8.3)
project(iai_image_logging)

add_compile_options(-std=c++11)

# OpenCV needs to be packaged seperately
#find_package(OpenCV 2 REQUIRED)
find_package(catkin REQUIRED
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
        iai_image_logging_msgs
        )

include_directories(${catkin_INCLUDE_DIRS}
#        ${OpenCV_INCLUDE_DIRS}
        )
add_library(storage_sub
        src/storage_sub.cpp
        src/global_vars.h
        src/connector.cpp src/connector.h)
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
#        ${OpenCV_LIBRARIES}
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
