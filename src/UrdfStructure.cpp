//
// Created by rgrandia on 25.09.19.
//

#include "urdf2robcogen/UrdfStructure.hpp"

std::string toString(JointType jointType) {
  switch (jointType) {
    case JointType::fixed: {
      return "fixed";
    }
    case JointType::p_joint: {
      return "p_joint";
    }
    case JointType::r_joint: {
      return "r_joint";
    }
    default:
      throw std::runtime_error("Unknown joint type");
  }
}

std::string printChildren(const UrdfStructure& urdfStructure, const std::string& linkName, std::string tabs) {
  const auto& linkInfo = urdfStructure.links_.at(linkName);

  std::ostringstream out;
  if (!linkInfo.parentJoint_.empty()) {
    const auto& parentJointInfo = urdfStructure.joints_.at(linkInfo.parentJoint_);
    out << tabs << linkInfo.parentJoint_ << " <" << toString(parentJointInfo.type_) << "> \n";
    tabs = tabs + "\t";
  }

  out << tabs << linkName << " [link] \n";
  tabs = tabs + "\t";

  for (auto& frame : urdfStructure.frames_) {
    if (frame.second.parentLinkName_ == linkName) {
      out << tabs << "- " << frame.first << " (frame) \n";
    }
  }

  for (const auto& childName : linkInfo.childLinks_) {
    out << printChildren(urdfStructure, childName, tabs);
  }

  return out.str();
}

void printLinkJointStructure(const UrdfStructure& urdfStructure) {
  auto str = printChildren(urdfStructure, urdfStructure.rootLinkName_, "");
  std::cout << str << std::endl;
}
