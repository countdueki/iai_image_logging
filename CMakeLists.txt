cmake_minimum_required(VERSION 2.8.3)
project(iai_image_logging)

#add_compile_options(-std=c++11 -Wall -Wextra -Wstrict-aliasing -pedantic -fmax-errors=5 -Werror -Wunreachable-code -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op -Wmissing-include-dirs -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 -Wswitch-default -Wundef -Wno-unused -Wno-variadic-macros -Wno-parentheses -fdiagnostics-show-option)
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

add_library(image_logger_lib
        src/image_logger.cpp
        )

add_executable(image_logger
        src/image_logger.cpp
      ) # src/configurator.cpp src/configurator.h src/preprocessor.cpp src/preprocessor.h src/logger.cpp src/logger.h

 target_link_libraries(image_logger
   ${catkin_LIBRARIES}
 )

