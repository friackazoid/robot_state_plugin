cmake_minimum_required(VERSION 3.5)
project(robot_state_plugin)

#add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread date_time system filesystem)
find_package(PkgConfig REQUIRED)

# Ogre
#pkg_check_modules(OGRE OGRE)
#link_directories( ${OGRE_LIBRARY_DIRS} )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(class_loader)
find_package(moveit_ros_planning_interface)

#find_package(catkin REQUIRED COMPONENTS
    #class_loader
  #moveit_ros_planning_interface
  #pluginlib
  #roscpp
  #  rviz
  #)

  #get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#    if(_variableName MATCHES "^moveit")
#        message("${_variableName}=${${_variableName}}")
#    endif()
#endforeach()

#message(MOVEIT_LIB_NAME="${MOVEIT_LIB_NAME}")

# Qt Stuff
#if(rviz_QT_VERSION VERSION_LESS "5")
#	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
#	include(${QT_USE_FILE})
#  macro(qt_wrap_ui)
#    qt4_wrap_ui(${ARGN})
#  endmacro()
#else()
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
endmacro()
#endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

#catkin_package(
#  LIBRARIES
#    moveit_rviz_plugin_render_tools
#    moveit_robot_state_rviz_plugin_core
#    moveit_planning_scene_rviz_plugin_core
#  INCLUDE_DIRS
#    rviz_plugin_render_tools/include
#    robot_state_rviz_plugin/include
#  CATKIN_DEPENDS
#    moveit_ros_planning_interface
#  DEPENDS
#)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  class_loader
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
)

include_directories(rviz_plugin_render_tools/include
                    robot_state_rviz_plugin/include
                    ${Boost_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})

add_subdirectory(rviz_plugin_render_tools)
add_subdirectory(robot_state_rviz_plugin)

pluginlib_export_plugin_description_file(rviz_common robot_state_rviz_plugin_description.xml) 

install(FILES
  robot_state_rviz_plugin_description.xml
  DESTINATION share)

install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

ament_package()
