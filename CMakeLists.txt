cmake_minimum_required(VERSION 2.8.3)
project(ladybug2)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    image_transport
    cv_bridge
    image_geometry
)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(LadybugUtil
    src/ladybugUtilities.cpp
)

target_link_libraries(LadybugUtil
    ${OpenCV_LIBS}
)

catkin_package(
    INCLUDE_DIRS include    
    LIBRARIES ladybug2
    CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs cv_bridge image_geometry 
    DEPENDS system_lib
    LIBRARIES LadybugUtil
)

install(TARGETS LadybugUtil
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ladybug2/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

add_executable(split_ladybug_node 
    src/publishSplitLadybug.cpp
)

target_link_libraries(split_ladybug_node 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    LadybugUtil
)

