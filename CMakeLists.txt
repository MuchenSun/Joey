cmake_minimum_required(VERSION 3.16)
project(Joey)

# compile as c++ 20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# disable GNU extensions so the program remains compliant
# with C++ standards and can also run on other systems
set(CMAKE_CXX_EXTENSIONS OFF)

# enable all warning messages
add_compile_options(-Wall -Wextra)

# add catkin to cmake prefix path
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")

# find packages
find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs geometry_msgs)

# generate catkin config files
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp sensor_msgs geometry_msgs)

# include head files
include_directories(include
    ${catkin_INCLUDE_DIRS})

# declare library
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}/${PROJECT_NAME}.cpp)

# declare executable (ros node)
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME})

# link libraries
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES})
