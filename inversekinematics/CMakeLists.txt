include_directories(${catkin_INCLUDE_DIRS})
add_executable(inversekinematics node/ros.cpp node/ros.hpp common/Strategy.cpp common/Strategy.hpp classInfo/classinfo.cpp classInfo/classinfo.hpp)
target_link_libraries(inversekinematics ${catkin_LIBRARIES})
install(TARGETS inversekinematics
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



