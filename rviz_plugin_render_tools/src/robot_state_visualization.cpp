/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/rviz_plugin_render_tools/planning_link_updater.h>
#include <QApplication>

namespace moveit_rviz_plugin
{
RobotStateVisualization::RobotStateVisualization(Ogre::SceneNode* root_node, rviz_common::DisplayContext* context,
                                                 const std::string& name, rviz_common::properties::Property* parent_property)
  : robot_(root_node, context, name, parent_property)
//   , octree_voxel_render_mode_(OCTOMAP_OCCUPIED_VOXELS)
//   , octree_voxel_color_mode_(OCTOMAP_Z_AXIS_COLOR)
  , visible_(true)
  , visual_visible_(true)
  , collision_visible_(false)
{
  default_attached_object_color_.r = 0.0f;
  default_attached_object_color_.g = 0.7f;
  default_attached_object_color_.b = 0.0f;
  default_attached_object_color_.a = 1.0f;

}

void RobotStateVisualization::load(const urdf::ModelInterface& descr, bool visual, bool collision)
{
  // clear previously loaded model
  clear();

  robot_.load(descr, visual, collision);
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
  QApplication::processEvents();
}

void RobotStateVisualization::clear()
{
  robot_.clear();
}

void RobotStateVisualization::setDefaultAttachedObjectColor(const std_msgs::msg::ColorRGBA& default_attached_object_color)
{
  default_attached_object_color_ = default_attached_object_color;
}

void RobotStateVisualization::update(const moveit::core::RobotStateConstPtr& kinematic_state)
{
  updateHelper(kinematic_state, default_attached_object_color_, NULL);
}

void RobotStateVisualization::update(const moveit::core::RobotStateConstPtr& kinematic_state,
                                     const std_msgs::msg::ColorRGBA& default_attached_object_color)
{
  updateHelper(kinematic_state, default_attached_object_color, NULL);
}

void RobotStateVisualization::update(const moveit::core::RobotStateConstPtr& kinematic_state,
                                     const std_msgs::msg::ColorRGBA& default_attached_object_color,
                                     const std::map<std::string, std_msgs::msg::ColorRGBA>& color_map)
{
  updateHelper(kinematic_state, default_attached_object_color, &color_map);
}

void RobotStateVisualization::updateHelper(const moveit::core::RobotStateConstPtr& kinematic_state,
                                           const std_msgs::msg::ColorRGBA& ,
                                           const std::map<std::string, std_msgs::msg::ColorRGBA>* )
{
  robot_.update(PlanningLinkUpdater(kinematic_state));

  //std::vector<const moveit::core::AttachedBody*> attached_bodies;
  //kinematic_state->getAttachedBodies(attached_bodies);

  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
}

void RobotStateVisualization::setVisible(bool visible)
{
  visible_ = visible;
  robot_.setVisible(visible);
}

void RobotStateVisualization::setVisualVisible(bool visible)
{
  visual_visible_ = visible;
  robot_.setVisualVisible(visible);
}

void RobotStateVisualization::setCollisionVisible(bool visible)
{
  collision_visible_ = visible;
  robot_.setCollisionVisible(visible);
}

void RobotStateVisualization::setAlpha(float alpha)
{
  robot_.setAlpha(alpha);
}
}
