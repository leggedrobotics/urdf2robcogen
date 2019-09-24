//
// Created by rgrandia on 23.09.19.
//

#pragma once

#include "urdf2robcogen/UrdfInfo.hpp"

void generateKindsl(const std::string& robotName, const urdf::Model& urdf, const UrdfInfo& urdfInfo,
                    const std::map<std::string, urdf::Joint>& fixedFrames);