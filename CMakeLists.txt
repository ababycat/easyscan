cmake_minimum_required(VERSION 2.8.3)
project(easyscan)

add_compile_options(-std=c++11)

set(Eigen3_INCLUDE_DIR 
    /usr/include/eigen3)
set(OpenCV_INCLUDE_DIRS 
    /usr/local/include
    /usr/local/include/opencv2)
set(OpenCV_LIBS_DIR 
    /usr/local/lib)
set(OpenCV_LIBS
    opencv_core
    opencv_imgcodecs
    opencv_highgui
    opencv_videoio
    opencv_calib3d
    opencv_imgproc
    opencv_features2d)    

set(calibrate_SRCS
    src/calibrate.cpp)
set(take_photos_SRCS
    src/take_photos.cpp
    src/util.cpp)
set(camera_SRCS
    src/camera.cpp
    src/util.cpp)
set(process_SRCS
    src/process.cpp)
set(camera_node_SRCS
    src/camera_node.cpp 
    src/util.cpp)
set(nvidia_camera_node_SRCS
    src/nvidia_camera_node.cpp
    src/util.cpp)
    
set(INCLUDE_DIRS
    include/${PROJECT_NAME})

#set(camera_HDRS
#    include/${PROJECT_NAME}/util.h)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cv_bridge
    image_transport
    std_msgs)

find_package(Eigen3 REQUIRED)

catkin_package(
#    INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
    ${INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIR} 
    ${OpenCV_INCLUDE_DIRS})

# camera
add_executable(camera ${camera_SRCS})
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})
#install(TARGETS camera
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS camera
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# take_photos
add_executable(take_photos ${take_photos_SRCS})
target_link_libraries(take_photos ${catkin_LIBRARIES} ${OpenCV_LIBS})

# camera node
add_executable(camera_node ${camera_node_SRCS})
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(nvidia_camera_node ${nvidia_camera_node_SRCS})
target_link_libraries(nvidia_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(calibrate ${calibrate_SRCS})
target_link_libraries(calibrate ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(process ${process_SRCS})
target_link_libraries(process ${catkin_LIBRARIES} ${OpenCV_LIBS})


# install(TARGETS calibrate
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#message(STATUS "********")
#message(STATUS ${CATKIN_PACKAGE_BIN_DESTINATION})
#catkin_install_python(PROGRAMS script/localize_QRcode.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


