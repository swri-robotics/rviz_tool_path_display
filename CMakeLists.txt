cmake_minimum_required(VERSION 2.8.3)
project(rviz_tool_path_display)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(catkin REQUIRED COMPONENTS rviz pluginlib)

catkin_package(CATKIN_DEPENDS rviz pluginlib)

include_directories(${catkin_INCLUDE_DIRS})

qt5_wrap_cpp(MOC_FILES src/rviz_tool_path_display.h)
add_library(${PROJECT_NAME} src/rviz_tool_path_display.cpp ${MOC_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
