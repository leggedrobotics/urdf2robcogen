//
// Created by rgrandia on 24.09.19.
//

#pragma once

#include <string>
#include <vector>

void generateDtDsl(const std::string& robotName, const std::string& rootName, const std::vector<std::string>& robotLinkNames,
                   const std::vector<std::string>& robotFixedFrameNames);