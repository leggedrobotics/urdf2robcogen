//
// Created by rgrandia on 23.09.19.
//

#pragma once

#include <ros/console.h>
#include <urdf/model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "urdf2robcogen/UrdfInfo.hpp"

std::map<std::string, urdf::Joint> parseFixedFrames(const urdf::Model& urdf, const UrdfInfo& urdfInfo);

void moveInertiaFromFixedLinksRecursive(const urdf::LinkSharedPtr& link);

void moveInertiaFromChildToParent(const urdf::LinkSharedPtr& child, const urdf::LinkSharedPtr& parent);

void attachFrameToLink(std::string frameName, std::map<std::string, urdf::Joint>& frames, const urdf::LinkConstSharedPtr& link,
                       Eigen::Quaterniond q, Eigen::Vector3d r);

void addComFrame(const urdf::Model& urdf, const std::string& linkName, std::map<std::string, urdf::Joint>& frames);