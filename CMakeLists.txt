cmake_minimum_required(VERSION 2.8.3)
project(iai_image_logging)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        roscpp
        dynamic_reconfigure
        iai_image_logging_msgs
        image_transport
        compressed_image_transport
        mongodb_store
        )

catkin_package(CATKIN_DEPENDS
        iai_image_logging_msgs
        )

include_directories(${catkin_INCLUDE_DIRS})

add_library(image_logger_lib
        src/image_logger.cpp
        )
add_executable(image_logger
        src/image_logger.cpp
        )
target_link_libraries(image_logger
        ${catkin_LIBRARIES}
        )

add_library(preprocessor_lib
        src/preprocessor.cpp
        )
add_executable(preprocessor
        src/preprocessor.cpp
        )
target_link_libraries(preprocessor
        ${catkin_LIBRARIES}
        )


add_library(logger_lib
        src/logger.cpp
        )
add_executable(logger
        src/logger.cpp
        )
target_link_libraries(logger
        ${catkin_LIBRARIES}
        )

