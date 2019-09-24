//
// Created by rgrandia on 23.09.19.
//

#pragma once

#include <tinyxml.h>
#include <urdf/model.h>
#include <ros/console.h>

#include "urdf2robcogen/UrdfInfo.hpp"

void parseUrdf(const std::string& robotName, const std::string& urdfpath);

void sortLinksRecursive(const urdf::LinkSharedPtr& link, const std::map<std::string, unsigned int>& linkId);

//! Gets the order in which links appear in the original urdf
std::map<std::string, unsigned int> getLinkIdMap(const TiXmlDocument& urdfXml);
