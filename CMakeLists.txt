cmake_minimum_required(VERSION 2.8.3)
project(force_observer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        cmake_modules
  roscpp
  rospy
  std_msgs
)

find_package (Eigen3 REQUIRED)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package (
        INCLUDE_DIRS include
        LIBRARIES force_observer
        CATKIN_DEPENDS roscpp
        #DEPENDS eigen3/Eigen
)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)



set(SRC_FILES
        src/observer.cpp
        src/imu.cpp
 )
set(HDR_FILES
        include/force_observer/observer.h
        include/force_observer/imu.h
     )

add_library(${PROJECT_NAME} ${SRC_FILES} ${HDR_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(run_observer src/run_observer.cpp)
#target_link_libraries(run_observer force_observer)


catkin_python_setup()