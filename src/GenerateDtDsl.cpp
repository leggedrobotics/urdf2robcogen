//
// Created by rgrandia on 24.09.19.
//

#include "urdf2robcogen/GenerateDtDsl.hpp"

#include "urdf2robcogen/Utils.hpp"

#include <sstream>

std::string list_frames(const std::vector<std::string>& linkAndFrameNames) {
  std::ostringstream out;

  // All frames, 4 per line
  int k = 0;
  out << "\t";
  for (const auto& name : linkAndFrameNames) {
    if (k > 3) {
      out << std::endl;
      out << "\t";
      k = 0;
    }
    out << "fr_" << name;
    if (name != linkAndFrameNames.back()) {
      out << ", ";
    }
    k++;
  }
  return out.str();
}

std::string list_transforms(const std::string& rootName, const std::vector<std::string>& linkAndFrameNames) {
  std::ostringstream out;

  // request all transforms from and to the base
  for (const auto& name : linkAndFrameNames) {
    if (name == rootName) {
      continue;
    }
    out << "\tbase=fr_" << rootName << ", target=fr_" << name << std::endl;
    out << "\tbase=fr_" << name << ", target=fr_" << rootName << std::endl;
  }

  return out.str();
}

std::string list_jacobians(const UrdfStructure& urdfStructure) {
  // Note: Don't request jacobians from frames of the same link!
  // For now, generate Jacobians to all links and frames from the base
  std::ostringstream out;

  for (const auto& link : urdfStructure.links_) {
    if (link.first != urdfStructure.rootLinkName_) {
      out << "\tbase=fr_" << urdfStructure.rootLinkName_ << ", target=fr_" << link.first << std::endl;
    }
  }

  for (const auto& frame : urdfStructure.frames_) {
    if (frame.second.parentLinkName_ != urdfStructure.rootLinkName_) {  // Skip frames that are rigidly attached to root
      out << "\tbase=fr_" << urdfStructure.rootLinkName_ << ", target=fr_" << frame.first << std::endl;
    }
  }

  return out.str();
}

std::string generateDtDsl(const UrdfStructure& urdfStructure) {
  std::ostringstream out;

  std::vector<std::string> linkAndFrameNames;
  auto linkNames = getKeys(urdfStructure.links_);
  auto frameNames = getKeys(urdfStructure.frames_);
  linkAndFrameNames.insert(linkAndFrameNames.end(), linkNames.begin(), linkNames.end());
  linkAndFrameNames.insert(linkAndFrameNames.end(), frameNames.begin(), frameNames.end());

  out << print_timeStamp();
  out << "Robot " << urdfStructure.robotName_ << std::endl;
  out << "Frames {" << std::endl;
  out << list_frames(linkAndFrameNames) << std::endl;
  out << "}" << std::endl;

  out << "Transforms {" << std::endl;
  out << list_transforms(urdfStructure.rootLinkName_, linkAndFrameNames) << std::endl;
  out << "}" << std::endl;

  out << "Jacobians {" << std::endl;
  out << list_jacobians(urdfStructure) << std::endl;
  out << "}" << std::endl;

  return out.str();
}