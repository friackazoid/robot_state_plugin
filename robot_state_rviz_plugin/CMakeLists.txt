
set(MOVEIT_LIB_NAME moveit_robot_state_rviz_plugin)

add_library(${MOVEIT_LIB_NAME}_core SHARED
  src/robot_state_display.cpp
  include/moveit/robot_state_rviz_plugin/robot_state_display.h)

#set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME}_core 
    moveit_rviz_plugin_render_tools
    #moveit_planning_scene_rviz_plugin_core 
    rviz_common::rviz_common
    #rviz_ogre_vendor::OgreMain
    #${OGRE_LIBRARIES} 
    #${QT_LIBRARIES} 
    #${Boost_LIBRARIES})
)

ament_target_dependencies(${MOVEIT_LIB_NAME}_core
    rclcpp
    rviz2
    moveit_ros_planning
    moveit_msgs
    pluginlib
    Boost
    rviz_ogre_vendor
)

target_include_directories(${MOVEIT_LIB_NAME}_core PRIVATE "${OGRE_PREFIX_DIR}/include")

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)

#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME} 
    ${MOVEIT_LIB_NAME}_core 
    #    moveit_planning_scene_rviz_plugin_core 
    #${Boost_LIBRARIES})
)

ament_target_dependencies(${MOVEIT_LIB_NAME}
    rclcpp
    pluginlib
    Boost
    rviz_ogre_vendor
)
target_include_directories(${MOVEIT_LIB_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include)#${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME}_core ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

