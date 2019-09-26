//
// Created by rgrandia on 25.09.19.
//

#pragma once

#include <string>

#include "urdf2robcogen/UrdfStructure.hpp"

std::string generateDtDsl(const UrdfStructure& urdfStructure);
