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
# set(nvidia_camera_node_SRCS
#     src/nvidia_camera_node.cpp
#     src/util.cpp)
    
set(INCLUDE_DIRS
    include/${PROJECT_NAME})

#set(camera_HDRS
#    include/${PROJECT_NAME}/util.h)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cv_bridge
    image_transport
    camera_calibration_parsers
    message_generation
    tf
    geometry_msgs
    std_msgs)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  npfloat32.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#    INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
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
#add_executable(camera_node ${camera_node_SRCS})
#target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(camera_node ${camera_node_SRCS})
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


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

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


