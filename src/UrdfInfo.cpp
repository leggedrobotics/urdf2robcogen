//
// Created by rgrandia on 23.09.19.
//

#include "urdf2robcogen/UrdfInfo.hpp"

UrdfInfo getUrdfInfo(const urdf::Model& urdf) {
  UrdfInfo urdfInfo;

  urdfInfo.rootLinkName_ = urdf.getRoot()->name;
  ROS_DEBUG_STREAM("The root of the urdf model is: " << urdfInfo.rootLinkName_);

  if (urdfInfo.rootLinkName_ == "world") {  // this is a fixed-base system, adjust root values to robot base
    ROS_DEBUG("We assume this is a fixed-base system.");
    if (urdf.getRoot()->child_links.size() != 1) {
      throw std::runtime_error("The world link should have exactly one child link.");
    }
    urdfInfo.floatingBaseSystem_ = false;
  }

  // Fill into temporaries to avoid aliasing with the urdfInfo, which is passed as constant reference.
  std::vector<std::string> robotLinkNames;
  std::vector<std::string> robotJointNames;
  std::vector<std::string> robotFixedFrameNames;
  collectJointsAndLinksRecursively(urdf.root_link_, urdfInfo, robotLinkNames, robotJointNames, robotFixedFrameNames);

  urdfInfo.robotLinkNames_ = std::move(robotLinkNames);
  urdfInfo.robotJointNames_ = std::move(robotJointNames);
  urdfInfo.robotFixedFrameNames_ = std::move(robotFixedFrameNames);

  urdfInfo.robotLinkParentNames_ = collectLinkParents(urdf, urdfInfo.robotLinkNames_, urdfInfo.rootLinkName_);
  urdfInfo.robotLinkChildrenNames_ = collectLinkChildren(urdfInfo.robotLinkNames_, urdfInfo.robotLinkParentNames_, urdfInfo.rootLinkName_);
  std::tie(urdfInfo.robotJointParentNames_, urdfInfo.robotLinkJointParentNames_) = collectJointKindslParents(urdf, urdfInfo.robotJointNames_);

  return urdfInfo;
}

void collectJointsAndLinksRecursively(const urdf::LinkSharedPtr& link, const UrdfInfo& urdfInfo, std::vector<std::string>& robotLinkNames,
                                      std::vector<std::string>& robotJointNames, std::vector<std::string>& robotFixedFrameNames) {
  /*
   * Operate on the kinematic tree from bottom (leaves) to root.
   */
  std::for_each(link->child_links.rbegin(), link->child_links.rend(), [&](const urdf::LinkSharedPtr& childLink) {
    collectJointsAndLinksRecursively(childLink, urdfInfo, robotLinkNames, robotJointNames, robotFixedFrameNames);
  });

  ROS_DEBUG_STREAM("[collectJointsAndLinksRecursively] -- operating on link: " << link->name);

  if (!link->parent_joint) {
    if (link->name != urdfInfo.rootLinkName_) {
      std::string msg = "[collectJointsAndLinksRecursively] ERROR: Encountered a link without a parent joint before reaching the root.";
      throw std::runtime_error(msg);
    }
    ROS_DEBUG("This is the root link. Adding it to the model.");
    robotLinkNames.push_back(link->name);

  } else {  // there exists a parent joint
    switch (link->parent_joint->type) {
      case urdf::Joint::FIXED: {
        robotFixedFrameNames.push_back(link->name);
        break;
      }
      case urdf::Joint::CONTINUOUS: {
        ROS_DEBUG_STREAM("Parent joint is revolute. Will add this link and joint " << link->parent_joint->name << " to the model.");
      }
      case urdf::Joint::REVOLUTE: {
        ROS_DEBUG_STREAM("Parent joint is revolute. Will add this link and joint " << link->parent_joint->name << " to the model.");
      }
      case urdf::Joint::PRISMATIC: {
        ROS_DEBUG_STREAM("Parent joint is prismatic. Will add this link and joint " << link->parent_joint->name << " to the model.");
        robotLinkNames.push_back(link->name);
        robotJointNames.push_back(link->parent_joint->name);
        break;
      }
      case urdf::Joint::FLOATING: {
        if (link->name != urdfInfo.rootLinkName_) {
          throw std::runtime_error("ERROR: Floating joint type encountered before reaching the root.");
        }
        break;
      }
      default: {
        std::cout << "ERROR: Unhandled joint type " << link->parent_joint->type << std::endl;
        throw std::runtime_error("Unhandled joint type");
      }
    }
  }
}

std::string getParentLinkName(const urdf::LinkConstSharedPtr& link) {
  if (!link->getParent()->parent_joint || link->getParent()->parent_joint->type != urdf::Joint::FIXED) {
    return link->getParent()->name;
  } else {
    return getParentLinkName(link->getParent());
  }
}

std::map<std::string, std::string> collectLinkParents(const urdf::Model& urdf, const std::vector<std::string>& robotLinkNames, const std::string& rootName) {
  std::map<std::string, std::string> linkParentMap;

  for (const std::string& linkName : robotLinkNames) {
    if (linkName != rootName){
      const auto& link = urdf.links_.at(linkName);
      const auto parentLinkName = getParentLinkName(link);
      ROS_DEBUG_STREAM("parent link of " << linkName << " is " << parentLinkName);
      linkParentMap.insert({linkName, parentLinkName});
    }
  }

  return linkParentMap;
}

std::map<std::string, std::vector<std::string>> collectLinkChildren(const std::vector<std::string>& robotLinkNames, const std::map<std::string, std::string>& linkParentMap, const std::string& rootName) {
  std::map<std::string, std::vector<std::string>> linkChildrenMap;

  // Initialize all links with an empty vector of children
  for (const std::string& linkName : robotLinkNames) {
    linkChildrenMap.insert({linkName, std::vector<std::string>()});
  }

  // Add each link to the children of its parent
  for (const std::string& linkName : robotLinkNames) {
    if (linkName != rootName){
      const auto& parentLinkName = linkParentMap.at(linkName);
      linkChildrenMap.at(parentLinkName).push_back(linkName);
    }
  }

  return linkChildrenMap;
}

std::pair<std::map<std::string, std::string>, std::map<std::string, std::string>> collectJointKindslParents(const urdf::Model& urdf, const std::vector<std::string>& robotJointNames) {
  std::map<std::string, std::string> jointKindslParents; //! first moving joint up the kinemenatic chain (=towards root)
  std::map<std::string, std::string> linkJointKindslParents; //! first moving joint up the kinemenatic chain (=towards root)

  // Add parents, root is his own parent
  jointKindslParents.insert({urdf.root_link_->name, urdf.root_link_->name});

  generateKindslParentsRecursive(robotJointNames, urdf.root_link_, urdf.root_link_->name, jointKindslParents, linkJointKindslParents);

  return {jointKindslParents, linkJointKindslParents};
}

void generateKindslParentsRecursive(const std::vector<std::string>& robotJointNames, const urdf::LinkSharedPtr& link, const std::string& parent,
    std::map<std::string, std::string>& jointKindslParent, std::map<std::string, std::string>& linkJointKindslParents) {
  if (!link) {
    throw std::runtime_error("link pointer not valid");
  }

  for (const auto& child : link->child_links) {
    const auto& joint = child->parent_joint;
    auto childName = child->name;
    auto jointName = joint->name;
    jointKindslParent.insert({jointName, parent});

    // verify it is a true joint and not a frame
    if (std::find(robotJointNames.begin(), robotJointNames.end(), jointName) == robotJointNames.end()) {
      // This child is not a moving joint -> traverse down
      linkJointKindslParents.insert({childName, parent});
      generateKindslParentsRecursive(robotJointNames, child, parent, jointKindslParent, linkJointKindslParents);
    } else {
      // Continue traversal with this child as parent
      linkJointKindslParents.insert({childName, jointName});
      generateKindslParentsRecursive(robotJointNames, child, jointName, jointKindslParent, linkJointKindslParents);
    }
  }
}

