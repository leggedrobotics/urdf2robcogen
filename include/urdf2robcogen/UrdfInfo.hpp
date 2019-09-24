//
// Created by rgrandia on 23.09.19.
//

#pragma once

#include <ros/console.h>
#include <tinyxml.h>
#include <urdf/model.h>

struct UrdfInfo {
  bool floatingBaseSystem_;
  std::string rootLinkName_;
  std::vector<std::string> robotLinkNames_;        //! "true" links that will appear in the kindsl file
  std::map<std::string, std::string> robotLinkParentNames_;  //! "true" parent links as they will appear in the kindsl file
  std::map<std::string, std::vector<std::string>> robotLinkChildrenNames_;  //! "true" children links as they will appear in the kindsl file
  std::map<std::string, std::string> robotLinkJointParentNames_;  //! "true" joints parent for each link
  std::vector<std::string> robotJointNames_;       //! "true" joints that will appear in the kindsl file
  std::map<std::string, std::string> robotJointParentNames_;  //! "true" joints parent for each joint
  std::vector<std::string> robotFixedFrameNames_;  //! Links that are connected through fixed joints will appear as frames.
};

UrdfInfo getUrdfInfo(const urdf::Model& urdf);

void collectJointsAndLinksRecursively(const urdf::LinkSharedPtr& link, const UrdfInfo& urdfInfo, std::vector<std::string>& robotLinkNames,
                                      std::vector<std::string>& robotJointNames, std::vector<std::string>& robotFixedFrameNames);

std::string getParentLinkName(const urdf::LinkConstSharedPtr& link);

std::map<std::string, std::string> collectLinkParents(const urdf::Model& urdf, const std::vector<std::string>& robotLinkNames, const std::string& rootName);

std::map<std::string, std::vector<std::string>> collectLinkChildren(const std::vector<std::string>& robotLinkNames, const std::map<std::string, std::string>& linkParentMap, const std::string& rootName);

std::pair<std::map<std::string, std::string>, std::map<std::string, std::string>> collectJointKindslParents(const urdf::Model& urdf, const std::vector<std::string>& robotJointNames);

void generateKindslParentsRecursive(const std::vector<std::string>& robotJointNames, const urdf::LinkSharedPtr& link, const std::string& parent,
                                    std::map<std::string, std::string>& jointKindslParent, std::map<std::string, std::string>& linkJointKindslParents);
