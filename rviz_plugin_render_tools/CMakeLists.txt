set(MOVEIT_LIB_NAME moveit_rviz_plugin_render_tools)

#message(MOVEIT_LIB_NAME="${MOVEIT_LIB_NAME}")

set(HEADERS
  include/moveit/rviz_plugin_render_tools/planning_link_updater.h
  include/moveit/rviz_plugin_render_tools/robot_state_visualization.h
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_BUILD_DIR}/rviz_plugin_render_tools
)

add_library(${MOVEIT_LIB_NAME}
  src/robot_state_visualization.cpp
  src/planning_link_updater.cpp
  ${HEADERS}
)

#message(PROJECT_NAME_VERSION="${PROJECT_NAME}")

#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME}
    #  ${catkin_LIBRARIES}
    # moveit_rviz_plugin_render_tools
    #moveit_robot_state_rviz_plugin_core
    #moveit_planning_scene_rviz_plugin_core
  #  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
)

ament_target_dependencies(${MOVEIT_LIB_NAME}
    rclcpp
    moveit_core
    rviz2
    Boost
    rviz_ogre_vendor
    rviz_default_plugins
)
target_include_directories(${MOVEIT_LIB_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include)#${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
