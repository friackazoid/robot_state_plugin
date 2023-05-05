/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

//#include <moveit/robot_state_rviz_plugin/robot_state_display.h>

#include "moveit/robot_state_rviz_plugin/robot_state_display.h"
#include <memory>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>

//#include <rviz/visualization_manager.h>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
// #include <tf/transform_listener.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <srdfdom/model.h>

namespace moveit_rviz_plugin
{
// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
RobotStateDisplay::RobotStateDisplay() : Display(), update_state_(false), load_robot_model_(false)
{
  robot_description_property_ = new rviz_common::properties::StringProperty(
      "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
      this, SLOT(changedRobotDescription()), this);

  robot_state_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Robot State Topic", "display_robot_state", rosidl_generator_traits::data_type<moveit_msgs::msg::DisplayRobotState>(),
      "The topic on which the moveit_msgs::RobotState messages are received", this, SLOT(changedRobotStateTopic()), this);

  // Planning scene category -------------------------------------------------------------------------------------------
  root_link_name_property_ =
      new rviz_common::properties::StringProperty("Robot Root Link", "", "Shows the name of the root link for the robot model", this,
                               SLOT(changedRootLinkName()), this);
  root_link_name_property_->setReadOnly(true);

  robot_alpha_property_ = new rviz_common::properties::FloatProperty("Robot Alpha", 1.0f, "Specifies the alpha for the robot links", this,
                                                  SLOT(changedRobotSceneAlpha()), this);
  robot_alpha_property_->setMin(0.0);
  robot_alpha_property_->setMax(1.0);

  attached_body_color_property_ =
      new rviz_common::properties::ColorProperty("Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies", this,
                              SLOT(changedAttachedBodyColor()), this);

  enable_link_highlight_ =
      new rviz_common::properties::BoolProperty("Show Highlights", true, "Specifies whether link highlighting is enabled", this,
                             SLOT(changedEnableLinkHighlight()), this);
  enable_visual_visible_ =
      new rviz_common::properties::BoolProperty("Visual Enabled", true, "Whether to display the visual representation of the robot.", this,
                             SLOT(changedEnableVisualVisible()), this);
  enable_collision_visible_ = new rviz_common::properties::BoolProperty("Collision Enabled", false,
                                                     "Whether to display the collision representation of the robot.",
                                                     this, SLOT(changedEnableCollisionVisible()), this);

  show_all_links_ = new rviz_common::properties::BoolProperty("Show All Links", true, "Toggle all links visibility on or off.", this,
                                           SLOT(changedAllLinks()), this);

  highlight_link_ =
      new rviz_common::properties::StringProperty("Highlight Link", "", "Highlight chosen link", this,
                               SLOT(changedHighlightColor()), this);

  unhighlight_link_ =
      new rviz_common::properties::StringProperty("Unhighlight Link", "", "Unhighlight chosen link", this,
                               SLOT(changedUnhighlightColor()), this);

//   highlight_link_ =
//       new rviz::ColorProperty("Change Color Link Test", QColor(150, 50, 150), "Change Color Link Test", this,
//                               SLOT(changedHighlightColor()), this);


}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
RobotStateDisplay::~RobotStateDisplay()
{
}

void RobotStateDisplay::onInitialize()
{
  Display::onInitialize();
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    RVIZ_COMMON_LOG_WARNING("Unable to lock weak_ptr from DisplayContext in RobotStateDisplay constructor");
    return;
  }
  robot_state_topic_property_->initialize(ros_node_abstraction);
  root_nh_ = ros_node_abstraction->get_raw_node();
  robot_ = std::make_shared<RobotStateVisualization>(scene_node_, context_, "Robot State", this);
  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  robot_->setVisible(false);
}

void RobotStateDisplay::reset()
{
  robot_->clear();
  rdf_loader_.reset();
  Display::reset();

  loadRobotModel();
}

void RobotStateDisplay::changedAllLinks()
{
  Property* links_prop = subProp("Links");
  QVariant value(show_all_links_->getBool());

  for (int i = 0; i < links_prop->numChildren(); ++i)
  {
    Property* link_prop = links_prop->childAt(i);
    link_prop->setValue(value);
  }
}

void RobotStateDisplay::setHighlight(const std::string& link_name, const std_msgs::msg::ColorRGBA& color)
{
  rviz_default_plugins::robot::RobotLink* link = robot_->getRobot().getLink(link_name);
  if (link)
  {
    link->setColor(color.r, color.g, color.b);
    link->setRobotAlpha(color.a * robot_alpha_property_->getFloat());
  }
}

void RobotStateDisplay::unsetHighlight(const std::string& link_name)
{
  rviz_default_plugins::robot::RobotLink* link = robot_->getRobot().getLink(link_name);
  if (link)
  {
    link->unsetColor();
    link->setRobotAlpha(robot_alpha_property_->getFloat());
  }
}

void RobotStateDisplay::changedEnableLinkHighlight()
{
  if (enable_link_highlight_->getBool())
  {
    for (std::map<std::string, std_msgs::msg::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end(); ++it)
    {
      setHighlight(it->first, it->second);
    }
  }
  else
  {
    for (std::map<std::string, std_msgs::msg::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end(); ++it)
    {
      unsetHighlight(it->first);
    }
  }
}

void RobotStateDisplay::changedEnableVisualVisible()
{
  robot_->setVisualVisible(enable_visual_visible_->getBool());
}

void RobotStateDisplay::changedEnableCollisionVisible()
{
  robot_->setCollisionVisible(enable_collision_visible_->getBool());
}

void RobotStateDisplay::setRobotHighlights(const moveit_msgs::msg::DisplayRobotState::_highlight_links_type& highlight_links)
{
  if (highlight_links.empty() && highlights_.empty())
    return;

  std::map<std::string, std_msgs::msg::ColorRGBA> highlights;
  for (moveit_msgs::msg::DisplayRobotState::_highlight_links_type::const_iterator it = highlight_links.begin();
       it != highlight_links.end(); ++it)
  {
    highlights[it->id] = it->color;
  }

  if (enable_link_highlight_->getBool())
  {
    std::map<std::string, std_msgs::msg::ColorRGBA>::iterator ho = highlights_.begin();
    std::map<std::string, std_msgs::msg::ColorRGBA>::iterator hn = highlights.begin();
    while (ho != highlights_.end() || hn != highlights.end())
    {
      if (ho == highlights_.end())
      {
        setHighlight(hn->first, hn->second);
        ++hn;
      }
      else if (hn == highlights.end())
      {
        unsetHighlight(ho->first);
        ++ho;
      }
      else if (hn->first < ho->first)
      {
        setHighlight(hn->first, hn->second);
        ++hn;
      }
      else if (hn->first > ho->first)
      {
        unsetHighlight(ho->first);
        ++ho;
      }
      else if (hn->second != ho->second)
      {
        setHighlight(hn->first, hn->second);
        ++ho;
        ++hn;
      }
      else
      {
        ++ho;
        ++hn;
      }
    }
  }

  swap(highlights, highlights_);
}

void RobotStateDisplay::changedHighlightColor()
{
  if (robot_)
  {
    std_msgs::msg::ColorRGBA color_msg;
    color_msg.r = 255;
    color_msg.g = 0;
    color_msg.b = 0;
    color_msg.a = 0.7;
    setHighlight( highlight_link_->getStdString(), color_msg);
    update_state_ = true;
  }
}

void RobotStateDisplay::changedUnhighlightColor()
{
  if (robot_)
  {
    unsetHighlight(unhighlight_link_->getStdString());
    update_state_ = true;
  }
}

void RobotStateDisplay::changedAttachedBodyColor()
{
  if (robot_)
  {
    QColor color = attached_body_color_property_->getColor();
    std_msgs::msg::ColorRGBA color_msg;
    color_msg.r = color.redF();
    color_msg.g = color.greenF();
    color_msg.b = color.blueF();
    color_msg.a = robot_alpha_property_->getFloat();
    robot_->setDefaultAttachedObjectColor(color_msg);
    update_state_ = true;
  }
}

void RobotStateDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void RobotStateDisplay::changedRootLinkName()
{
}

void RobotStateDisplay::changedRobotSceneAlpha()
{
  if (robot_)
  {
    robot_->setAlpha(robot_alpha_property_->getFloat());
    QColor color = attached_body_color_property_->getColor();
    std_msgs::msg::ColorRGBA color_msg;
    color_msg.r = color.redF();
    color_msg.g = color.greenF();
    color_msg.b = color.blueF();
    color_msg.a = robot_alpha_property_->getFloat();
    robot_->setDefaultAttachedObjectColor(color_msg);
    update_state_ = true;
  }
}

void RobotStateDisplay::changedRobotStateTopic()
{

  // reset model to default state, we don't want to show previous messages
  if (static_cast<bool>(kstate_))
    kstate_->setToDefaultValues();
  update_state_ = true;

  robot_state_subscriber_ = root_nh_->create_subscription<moveit_msgs::msg::DisplayRobotState>(robot_state_topic_property_->getStdString(), 10,
        std::bind(&RobotStateDisplay::newRobotStateCallback, this, std::placeholders::_1));
}

void RobotStateDisplay::newRobotStateCallback(const moveit_msgs::msg::DisplayRobotState::ConstSharedPtr state_msg)
{
  if (!kmodel_)
    return;
  if (!kstate_)
    kstate_ = std::make_shared<moveit::core::RobotState>(kmodel_);
  // possibly use TF to construct a robot_state::Transforms object to pass in to the conversion functio?
  
  try
  {
     if (!moveit::core::isEmpty(state_msg->state))
        moveit::core::robotStateMsgToRobotState(state_msg->state, *kstate_);
     setRobotHighlights(state_msg->highlight_links);
     RCLCPP_DEBUG_STREAM(root_nh_->get_logger(), "My log message " << 4);
  }
  catch (const moveit::Exception& e)
  {
    kstate_->setToDefaultValues();
    setRobotHighlights(moveit_msgs::msg::DisplayRobotState::_highlight_links_type());
    setStatus(rviz_common::properties::StatusProperty::Error, "RobotState", e.what());
    robot_->setVisible(false);
    return;
  }
            
  //robot_state::robotStateMsgToRobotState(state_msg->state, *kstate_);

  if (robot_->isVisible() != !state_msg->hide)
  {
    robot_->setVisible(!state_msg->hide);
    if (robot_->isVisible())
      setStatus(rviz_common::properties::StatusProperty::Ok, "RobotState", "");
    else
      setStatus(rviz_common::properties::StatusProperty::Warn, "RobotState", "Hidden");
  }
  setRobotHighlights(state_msg->highlight_links);
  update_state_ = true;
}

void RobotStateDisplay::setLinkColor(const std::string& link_name, const QColor& color)
{
  setLinkColor(&robot_->getRobot(), link_name, color);
}

void RobotStateDisplay::unsetLinkColor(const std::string& link_name)
{
  unsetLinkColor(&robot_->getRobot(), link_name);
}

void RobotStateDisplay::setLinkColor(rviz_default_plugins::robot::Robot* robot, const std::string& link_name, const QColor& color)
{
    // auto ?
    rviz_default_plugins::robot::RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->setColor(color.redF(), color.greenF(), color.blueF());
}

void RobotStateDisplay::unsetLinkColor(rviz_default_plugins::robot::Robot* robot, const std::string& link_name)
{
    // auto
    rviz_default_plugins::robot::RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->unsetColor();
}

// ******************************************************************************************
// Load
// ******************************************************************************************
void RobotStateDisplay::loadRobotModel()
{
  load_robot_model_ = false;

  std::string urdf_str;
  rdf_loader::RDFLoader::loadXmlFileToString(urdf_str, robot_description_property_->getStdString(), {});

  if (!urdf_str.empty())
  {
    try {
        auto urdf = std::make_shared<urdf::Model>();
        urdf->initString(urdf_str);
        kmodel_ =  std::make_shared<moveit::core::RobotModel>(urdf, std::make_shared<srdf::Model>());
        robot_->load(*kmodel_->getURDF());

        kstate_ = std::make_shared<moveit::core::RobotState>(kmodel_);
        kstate_->setToDefaultValues();

        bool oldState = root_link_name_property_->blockSignals(true);
        root_link_name_property_->setStdString(getRobotModel()->getRootLinkName());
        root_link_name_property_->blockSignals(oldState);
        update_state_ = true;
        setStatus(rviz_common::properties::StatusProperty::Ok, "RobotState", "Planning Model Loaded Successfully");

        changedEnableVisualVisible();
        changedEnableCollisionVisible();
        robot_->setVisible(true);

    } catch (const moveit::Exception& e) {
        setStatus(rviz_common::properties::StatusProperty::Error, "RobotState", e.what());
        robot_->setVisible(false);
    }
  }
  else
    setStatus(rviz_common::properties::StatusProperty::Error, "RobotState", "No Planning Model Loaded");

  highlights_.clear();

}

void RobotStateDisplay::onEnable()
{
  Display::onEnable();
  load_robot_model_ = true;  // allow loading of robot model in update()
  calculateOffsetPosition();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void RobotStateDisplay::onDisable()
{
  //robot_state_subscriber_.reset();
  if (robot_)
    robot_->setVisible(false);
  Display::onDisable();
}

void RobotStateDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  if (load_robot_model_)
  {
    loadRobotModel();
    changedRobotStateTopic();
  }

  calculateOffsetPosition();
  if (robot_ && update_state_ && kstate_)
  {
    update_state_ = false;
    kstate_->update();
    robot_->update(kstate_);
  }
}

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void RobotStateDisplay::calculateOffsetPosition()
{
  if (!getRobotModel())
    return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(getRobotModel()->getModelFrame(), 
          rclcpp::Time(0, 0, context_->getClock()->get_clock_type()),
          position, orientation);

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void RobotStateDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
  calculateOffsetPosition();
}

}  // namespace moveit_rviz_plugin
