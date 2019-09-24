//
// Created by rgrandia on 23.09.19.
//

#include "urdf2robcogen/parseUrdf.hpp"

#include "urdf2robcogen/FixJointAxes.hpp"
#include "urdf2robcogen/ParseFixedFrames.hpp"
#include "urdf2robcogen/generateDtDsl.hpp"
#include "urdf2robcogen/generateKindsl.hpp"

#include <Eigen/Core>

void parseUrdf(const std::string& robotName, const std::string& urdfpath) {
  urdf::Model urdf;  //! urdf parser object
  if (!urdf.initFile(urdfpath)) {
    throw std::runtime_error("Invalid URDF File");
  }

  ROS_DEBUG("Parsing the urdf...");

  { // Sort the Urdf links according to the xml
    TiXmlDocument urdfXml;  //! handle to the urdf xml file
    urdfXml.LoadFile(urdfpath);
    auto linkId = getLinkIdMap(urdfXml);
    sortLinksRecursive(urdf.root_link_, linkId);
  }

  auto urdfInfo = getUrdfInfo(urdf);

  auto fixedFrames = parseFixedFrames(urdf, urdfInfo);

  FixJointAxes(urdf, urdfInfo, fixedFrames);

  generateKindsl(robotName, urdf, urdfInfo, fixedFrames);
  generateDtDsl(robotName, urdfInfo.rootLinkName_, urdfInfo.robotLinkNames_, urdfInfo.robotFixedFrameNames_);
}

void sortLinksRecursive(const urdf::LinkSharedPtr& link, const std::map<std::string, unsigned int>& linkId) {
  std::sort(link->child_links.begin(), link->child_links.end(), [&linkId](urdf::LinkSharedPtr& a, urdf::LinkSharedPtr& b) {
    return linkId.at(a->name) < linkId.at(b->name);
  });
  std::for_each(link->child_links.rbegin(), link->child_links.rend(),
                [&linkId](urdf::LinkSharedPtr& childLink) { sortLinksRecursive(childLink, linkId); });
}

std::map<std::string, unsigned int> getLinkIdMap(const TiXmlDocument& urdfXml) {
  const auto urdf_xml_root = urdfXml.RootElement();

  std::map<std::string, unsigned int> linkIdMap;
  ROS_DEBUG("Ordered list of URDF links:");
  for (auto child = urdf_xml_root->FirstChildElement("link"); child; child = child->NextSiblingElement("link")) {
    std::string linkName;
    if (child->QueryStringAttribute("name", &linkName) == TIXML_SUCCESS) {
      linkIdMap.insert(std::pair<std::string, unsigned int>(linkName, linkIdMap.size() + 1));
      ROS_DEBUG_STREAM(linkName);
    }
  }
  ROS_DEBUG("--- End of list. ---");
  return linkIdMap;
}