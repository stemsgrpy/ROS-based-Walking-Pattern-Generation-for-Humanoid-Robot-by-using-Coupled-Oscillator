include_directories(${catkin_INCLUDE_DIRS})
add_executable(walkingcycle node/ros.cpp node/ros.hpp common/Strategy.cpp common/Strategy.hpp classInfo/classinfo.cpp classInfo/classinfo.hpp)
target_link_libraries(walkingcycle ${catkin_LIBRARIES})
install(TARGETS walkingcycle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
