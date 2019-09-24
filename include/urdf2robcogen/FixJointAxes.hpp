//
// Created by rgrandia on 23.09.19.
//

#pragma once

#include <ros/console.h>
#include <urdf/model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "urdf2robcogen/UrdfInfo.hpp"

void FixJointAxes(const urdf::Model& urdf, const UrdfInfo& urdfInfo, std::map<std::string, urdf::Joint>& fixedFrames);

std::map<std::string, urdf::Pose> getJointPosesInRoot(const urdf::Model& urdf, const UrdfInfo& urdfInfo);

void generateJointPosesInRootRecursive(const urdf::LinkSharedPtr& link, const urdf::Pose& parentPoseInRoot,
                                       std::map<std::string, urdf::Pose>& jointPosesInRoot);

void FixJointAxesRecursive(const UrdfInfo& urdfInfo, const urdf::LinkSharedPtr& link,
                           const std::map<std::string, std::string>& jointKindslParent, std::map<std::string, urdf::Pose>& jointPosesInRoot,
                           std::map<std::string, urdf::Joint>& fixedFrames);