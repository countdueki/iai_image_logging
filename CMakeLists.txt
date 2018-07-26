cmake_minimum_required(VERSION 2.8.3)
project(iai_image_logging)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        roscpp
        dynamic_reconfigure
        iai_image_logging_msgs
        image_transport
        mongodb_store
        mongodb_log

        )

catkin_package(
          CATKIN_DEPENDS  iai_image_logging_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_library(image_logger_lib src/image_logger.cpp src/configurator.cpp src/preprocessor.cpp )

add_executable(image_logger src/image_logger.cpp src/configurator.cpp ) # src/configurator.cpp src/configurator.h src/preprocessor.cpp src/preprocessor.h src/logger.cpp src/logger.h

 target_link_libraries(image_logger
   ${catkin_LIBRARIES}
 )

add_executable(logger src/logger.cpp ) # src/configurator.cpp src/configurator.h src/preprocessor.cpp src/preprocessor.h src/logger.cpp src/logger.h

target_link_libraries(logger
        ${catkin_LIBRARIES}
        )
