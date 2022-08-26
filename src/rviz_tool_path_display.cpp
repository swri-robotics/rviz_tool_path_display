/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "rviz_tool_path_display.h"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/validate_floats.h>
#include <rviz/validate_quaternions.h>

namespace
{
Ogre::Vector3 vectorRosToOgre(geometry_msgs::Point const& point)
{
  return Ogre::Vector3(point.x, point.y, point.z);
}

Ogre::Quaternion quaternionRosToOgre(geometry_msgs::Quaternion const& quaternion)
{
  Ogre::Quaternion q;
  rviz::normalizeQuaternion(quaternion, q);
  return q;
}
}  // namespace

namespace rviz
{
ToolPathDisplay::ToolPathDisplay()
{
  axes_length_property_ =
      new FloatProperty("Axes Length", 0.3, "Length of each axis, in meters.", this, SLOT(updateAxesGeometry()));

  axes_radius_property_ =
      new FloatProperty("Axes Radius", 0.01, "Radius of each axis, in meters.", this, SLOT(updateAxesGeometry()));

  axes_visibility_property_ = new BoolProperty("Show Axes", true, "Toggles the visibility of the axes display", this,
                                               SLOT(updateAxesVisibility()));
  pts_visibility_property_ = new BoolProperty("Show Points", true, "Toggles the visibility of the points display", this,
                                              SLOT(updatePtsVisibility()));
  lines_visibility_property_ = new BoolProperty("Show Lines", true, "Toggles the visibiltiy of the lines display", this,
                                                SLOT(updateLinesVisibility()));
}

ToolPathDisplay::~ToolPathDisplay()
{
  if (initialized())
  {
    scene_manager_->destroyManualObject(pts_object_);
    scene_manager_->destroyManualObject(lines_object_);
  }
}

void ToolPathDisplay::onInitialize()
{
  MFDClass::onInitialize();
  axes_node_ = scene_node_->createChildSceneNode();

  pts_object_ = scene_manager_->createManualObject();
  scene_node_->attachObject(pts_object_);

  lines_object_ = scene_manager_->createManualObject();
  scene_node_->attachObject(lines_object_);

  updateDisplay();
}

bool validateFloats(const geometry_msgs::PoseArray& msg)
{
  return validateFloats(msg.poses);
}

void ToolPathDisplay::processMessage(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  if (!validateFloats(*msg))
  {
    setStatus(StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  if (!validateQuaternions(msg->poses))
  {
    ROS_WARN_ONCE_NAMED("quaternions",
                        "PoseArray msg received on topic '%s' contains unnormalized quaternions. "
                        "This warning will only be output once but may be true for others; "
                        "enable DEBUG messages for ros.rviz.quaternions to see more details.",
                        topic_property_->getTopicStd().c_str());
    ROS_DEBUG_NAMED("quaternions", "PoseArray msg received on topic '%s' contains unnormalized quaternions.",
                    topic_property_->getTopicStd().c_str());
  }

  if (!setTransform(msg->header))
  {
    setStatus(StatusProperty::Error, "Topic", "Failed to look up transform");
    return;
  }

  poses_.resize(msg->poses.size());
  for (std::size_t i = 0; i < msg->poses.size(); ++i)
  {
    poses_[i].position = vectorRosToOgre(msg->poses[i].position);
    poses_[i].orientation = quaternionRosToOgre(msg->poses[i].orientation);
  }

  updateDisplay();
  context_->queueRender();
}

bool ToolPathDisplay::setTransform(std_msgs::Header const& header)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(header, position, orientation))
  {
    ROS_ERROR("Error transforming pose '%s' from frame '%s' to frame '%s'", qPrintable(getName()),
              header.frame_id.c_str(), qPrintable(fixed_frame_));
    return false;
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
  return true;
}

void ToolPathDisplay::updateDisplay()
{
  updateAxes();
  updatePoints();
  updateLines();
}

void ToolPathDisplay::updateAxes()
{
  while (axes_.size() < poses_.size())
    axes_.push_back(makeAxes());
  while (axes_.size() > poses_.size())
    axes_.pop_back();
  for (std::size_t i = 0; i < poses_.size(); ++i)
  {
    axes_[i].setPosition(poses_[i].position);
    axes_[i].setOrientation(poses_[i].orientation);
  }

  axes_node_->setVisible(axes_visibility_property_->getBool());
}

Axes* ToolPathDisplay::makeAxes()
{
  return new Axes(scene_manager_, axes_node_, axes_length_property_->getFloat(), axes_radius_property_->getFloat());
}

void ToolPathDisplay::updatePoints()
{
  if (poses_.empty())
    return;

  pts_object_->clear();
  pts_object_->estimateVertexCount(poses_.size());
  pts_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
  for (const OgrePose& pose : poses_)
  {
    pts_object_->position(pose.position);
  }
  pts_object_->end();

  pts_object_->setVisible(pts_visibility_property_->getBool());
}

void ToolPathDisplay::updateLines()
{
  if (poses_.empty())
    return;

  lines_object_->clear();
  lines_object_->estimateIndexCount(poses_.size());
  lines_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  for (unsigned i = 0; i < poses_.size(); ++i)
  {
    lines_object_->position(poses_[i].position);
    lines_object_->index(i);
  }
  lines_object_->end();

  lines_object_->setVisible(lines_visibility_property_->getBool());
}

void ToolPathDisplay::reset()
{
  MFDClass::reset();
  axes_.clear();
  pts_object_->clear();
  lines_object_->clear();
}

void ToolPathDisplay::updateAxesGeometry()
{
  for (std::size_t i = 0; i < poses_.size(); ++i)
  {
    axes_[i].set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
  }
  context_->queueRender();
}

void ToolPathDisplay::updateAxesVisibility()
{
  axes_node_->setVisible(axes_visibility_property_->getBool());
}

void ToolPathDisplay::updatePtsVisibility()
{
  pts_object_->setVisible(pts_visibility_property_->getBool());
}

void ToolPathDisplay::updateLinesVisibility()
{
  lines_object_->setVisible(lines_visibility_property_->getBool());
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::ToolPathDisplay, rviz::Display)
