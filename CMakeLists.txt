##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(walkinggait)
find_package(catkin)##

##############################################################################
# Catkin Configuration
##############################################################################

# These find_package calls setup catkin_INCLUDE_DIRS and ROS_LIBRARIES and 
# all the component variables too (e.g. roscpp_INCLUDE_DIR?)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)
find_package(OpenCV)

include_directories(${catkin_INCLUDE_DIRS})

##############################################################################
# Services
##############################################################################

add_message_files(DIRECTORY msg FILES Interface.msg Test.msg)
add_service_files(DIRECTORY srv FILES Update.srv Serial.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs roscpp)

##############################################################################
# Subprojects
##############################################################################

foreach(subdir

walkingcycle
walkingtrajectory
inversekinematics

)
  #install(DIRECTORY ${subdir} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_ros_pkg)
  add_subdirectory(${subdir})
endforeach()

##############################################################################
# Optional Installs - these aren't really necessary
##############################################################################

install(DIRECTORY resources DESTINATION ${CMAKE_INSTALL_PREFIX}/share/walkinggait)
install(DIRECTORY ui DESTINATION ${CMAKE_INSTALL_PREFIX}/share/walkinggait)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

