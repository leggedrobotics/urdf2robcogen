//
// Created by rgrandia on 24.09.19.
//

#include "urdf2robcogen/Urdf2RobCoGen.hpp"

#include "urdf2robcogen/GenerateDtDsl.hpp"
#include "urdf2robcogen/GenerateKindsl.hpp"
#include "urdf2robcogen/UrdfStructure.hpp"

#include "urdf2robcogen/Utils.hpp"

#include <tinyxml.h>
#include <urdf/model.h>

#include <algorithm>
#include <fstream>

LinkInfo getLinkInfo(const urdf::Link& link, const UrdfStructure& urdfStructure) {
  LinkInfo linkInfo;

  // Id
  linkInfo.id = urdfStructure.links_.size();

  // ParentJoint
  if (link.name != urdfStructure.rootLinkName_) {
    linkInfo.parentJoint_ = link.parent_joint->name;
  }

  // Get Children
  for (const auto& child : link.child_links) {
    linkInfo.childLinks_.push_back(child->name);
  }

  // Set Inertia
  if (link.inertial) {
    std::unique_ptr<Inertia> inertia(new Inertia());
    inertia->m_ = link.inertial->mass;
    inertia->I_ = inertiaMatrixFromLink(link);

    // Get pose of parent joint
    PoseInWorld parentJointPose;
    if (link.name == urdfStructure.rootLinkName_) {
      parentJointPose.position_ = Eigen::Vector3d::Zero();
      parentJointPose.rotationWorldToFrame_ = Eigen::Quaterniond::Identity();
    } else {
      parentJointPose = urdfStructure.joints_.at(linkInfo.parentJoint_).poseInWorld_;
    }

    // Orientation
    const Eigen::Quaterniond q_L_C = toEigen(link.inertial->origin.rotation);  // rotation Com To LinkFrame
    const Eigen::Quaterniond q_L_W = parentJointPose.rotationWorldToFrame_;
    const Eigen::Quaterniond q_C_W = q_L_C.inverse() * q_L_W;  // rotation world to Com
    inertia->comFramePoseInWorld_.rotationWorldToFrame_ = q_C_W;

    // Position
    const Eigen::Vector3d L_p_C_L = toEigen(link.inertial->origin.position);  // position linkFrame to Com
    const Eigen::Vector3d W_p_L_W = parentJointPose.position_;
    const Eigen::Vector3d W_p_C_W = q_L_W.inverse().toRotationMatrix() * L_p_C_L + W_p_L_W;
    inertia->comFramePoseInWorld_.position_ = W_p_C_W;

    linkInfo.inertia_ = std::move(inertia);
  } else {
    if (link.name == urdfStructure.rootLinkName_) {
      // If root has no inertia in urdf, we need to add zero inertia to the root at root location to later properly accumulate inertias
      // through the tree.
      std::unique_ptr<Inertia> inertia(new Inertia());
      inertia->m_ = 0.0;
      inertia->I_.setZero();
      inertia->comFramePoseInWorld_.position_ = Eigen::Vector3d::Zero();
      inertia->comFramePoseInWorld_.rotationWorldToFrame_ = Eigen::Quaterniond::Identity();
      linkInfo.inertia_ = std::move(inertia);
    }
  }

  return linkInfo;
}

JointInfo getJointInfo(const urdf::Joint& joint, const UrdfStructure& urdfStructure) {
  JointInfo jointInfo;

  // Id
  jointInfo.id = urdfStructure.joints_.size();

  // Get parent link
  jointInfo.parentLink_ = joint.parent_link_name;

  // Get joint type
  switch (joint.type) {
    case urdf::Joint::FIXED: {
      jointInfo.type_ = JointType::fixed;
      break;
    }
    case urdf::Joint::CONTINUOUS:
    case urdf::Joint::REVOLUTE: {
      jointInfo.type_ = JointType::r_joint;
      break;
    }
    case urdf::Joint::PRISMATIC: {
      jointInfo.type_ = JointType::p_joint;
      break;
    }
    case urdf::Joint::FLOATING: {
      if (joint.parent_link_name != urdfStructure.rootLinkName_) {
        throw std::runtime_error("ERROR: Floating joint type encountered before reaching the root.");
      }
      break;
    }
    default: {
      throw std::runtime_error("Unhandled joint type");
    }
  }

  // Get pose in world
  PoseInWorld parentLinkPose;
  if (jointInfo.parentLink_ == urdfStructure.rootLinkName_) {
    parentLinkPose.position_ = Eigen::Vector3d::Zero();
    parentLinkPose.rotationWorldToFrame_ = Eigen::Quaterniond::Identity();
  } else {
    const auto& parentJoint = urdfStructure.links_.at(jointInfo.parentLink_).parentJoint_;
    parentLinkPose = urdfStructure.joints_.at(parentJoint).poseInWorld_;
  }

  // Orientation
  const Eigen::Quaterniond q_L_J = toEigen(joint.parent_to_joint_origin_transform.rotation);  // rotation joint To parent Link
  const Eigen::Quaterniond q_L_W = parentLinkPose.rotationWorldToFrame_;
  const Eigen::Quaterniond q_J_W = q_L_J.inverse() * q_L_W;  // rotation world to Com
  jointInfo.poseInWorld_.rotationWorldToFrame_ = q_J_W;

  // Position
  const Eigen::Vector3d L_p_J_L = toEigen(joint.parent_to_joint_origin_transform.position);  // position linkFrame to Com
  const Eigen::Vector3d W_p_L_W = parentLinkPose.position_;
  const Eigen::Vector3d W_p_J_W = q_L_W.inverse().toRotationMatrix() * L_p_J_L + W_p_L_W;
  jointInfo.poseInWorld_.position_ = W_p_J_W;

  // Axis
  jointInfo.axisInJointFrame_ = toEigen(joint.axis);

  return jointInfo;
}

void fillUrdfStructureRecursive(const urdf::Link& link, UrdfStructure& urdfStructure) {
  // Add current link
  urdfStructure.links_.insert({link.name, getLinkInfo(link, urdfStructure)});

  for (const auto& child : link.child_links) {
    // Add joints in the order defined by the child links
    const auto& joint = child->parent_joint;
    urdfStructure.joints_.insert({joint->name, getJointInfo(*joint, urdfStructure)});

    // Continue recursion
    fillUrdfStructureRecursive(*child, urdfStructure);
  }
}

void moveInertiaFromTo(LinkInfo& linkInfoA, LinkInfo& linkInfoB) {
  // moves inertia from A to B
  if (!linkInfoA.inertia_) {
    return;
  }

  if (!linkInfoB.inertia_) {
    linkInfoB.inertia_ = std::move(linkInfoA.inertia_);
    return;
  }

  const auto& comPoseInWorldA = linkInfoA.inertia_->comFramePoseInWorld_;
  const auto& comPoseInWorldB = linkInfoB.inertia_->comFramePoseInWorld_;

  // Masses
  double massA = linkInfoA.inertia_->m_;
  double massB = linkInfoB.inertia_->m_;
  double massTot = massA + massB;

  // New center of mass position
  PoseInWorld newComPose;
  newComPose.position_ = massA / massTot * comPoseInWorldA.position_ + massB / massTot * comPoseInWorldB.position_;
  newComPose.rotationWorldToFrame_ = comPoseInWorldB.rotationWorldToFrame_;

  Eigen::Matrix3d I_ofA_inNewCom = expressInertiaFromFrameAInComFrame(linkInfoA.inertia_->I_, massA, comPoseInWorldA, newComPose);
  Eigen::Matrix3d I_ofB_inNewCom = expressInertiaFromFrameAInComFrame(linkInfoB.inertia_->I_, massB, comPoseInWorldB, newComPose);

  // All inertia is assigned to B with the new com location
  linkInfoB.inertia_->comFramePoseInWorld_ = newComPose;
  linkInfoB.inertia_->m_ = massTot;
  linkInfoB.inertia_->I_ = I_ofA_inNewCom + I_ofB_inNewCom;

  // A will no longer have inertia
  linkInfoA.inertia_.reset();
}

void turnLinkIntoFrame(const std::string& linkName, UrdfStructure& urdfStructure) {
  auto& linkInfo = urdfStructure.links_.at(linkName);
  auto parentJoint = linkInfo.parentJoint_;

  if (linkInfo.parentJoint_.empty()) {
    throw std::runtime_error("Cannot turn a link into a frame if it does not have a parent joint");
  }

  const auto& parentJointInfo = urdfStructure.joints_.at(parentJoint);
  const auto& parentLink = parentJointInfo.parentLink_;
  auto& parentLinkInfo = urdfStructure.links_.at(parentLink);

  // Add link as a frame at the pose of the parent joint
  FrameInfo frameInfo;
  frameInfo.parentLinkName_ = parentLink;
  frameInfo.poseInWorld_ = parentJointInfo.poseInWorld_;
  urdfStructure.frames_.insert({linkName, frameInfo});

  // All joints attached to this link are re-attached to the parentLink
  for (const auto& childName : linkInfo.childLinks_) {
    const auto& childLinkInfo = urdfStructure.links_.at(childName);
    auto& childJointInfo = urdfStructure.joints_.at(childLinkInfo.parentJoint_);
    childJointInfo.parentLink_ = parentLink;
    parentLinkInfo.childLinks_.push_back(childName);
  }

  // All frames attached to this link are re-attached to the parentLink
  for (auto& frame : urdfStructure.frames_) {
    if (frame.second.parentLinkName_ == linkName) {
      frame.second.parentLinkName_ = parentLink;
    }
  }

  // Add Inertia from link to the parent link
  moveInertiaFromTo(linkInfo, parentLinkInfo);

  // remove the link from children of the parent
  removeFromContainer(linkName, parentLinkInfo.childLinks_);

  // remove the joint referencing this link as a parent
  removeFromContainer(parentJoint, urdfStructure.joints_);

  // remove the link from list of links
  removeFromContainer(linkName, urdfStructure.links_);
}

Inertia getFullInertiaInWorld(const UrdfStructure& urdfStructure) {
  auto rootInfo = urdfStructure.links_.at(urdfStructure.rootLinkName_);  // Copy of the root link
  for (auto link : urdfStructure.links_) {                               // makes copy of each link
    if (link.first != urdfStructure.rootLinkName_) {
      moveInertiaFromTo(link.second, rootInfo);
    }
  }
  return *rootInfo.inertia_;
}

bool hasFixedJoint(const UrdfStructure& urdfStructure) {
  for (const auto& joint : urdfStructure.joints_) {
    if (joint.second.type_ == JointType::fixed) {
      return true;
    }
  }
  return false;
}

void removeFixedJoints(UrdfStructure& urdfStructure, bool verbose) {
  while (hasFixedJoint(urdfStructure)) {
    // make copy of current links to avoid alias with the removing of links
    std::vector<std::string> linkNames = getKeys(urdfStructure.links_);
    for (const auto& linkName : linkNames) {
      if (linkName != urdfStructure.rootLinkName_) {
        const auto& linkInfo = urdfStructure.links_.at(linkName);
        if (urdfStructure.joints_.at(linkInfo.parentJoint_).type_ == JointType::fixed) {
          if (verbose) {
            std::cout << "Converting link '" << linkName << "' into a frame" << std::endl;
          }
          turnLinkIntoFrame(linkName, urdfStructure);
        }
      }
    }
  }
}

void rotateJointAxesToZaxis(UrdfStructure& urdfStructure, bool verbose) {
  Eigen::Vector3d desiredAxisInJointFrame{0.0, 0.0, 1.0};
  for (auto& joint : urdfStructure.joints_) {
    const auto& jointName = joint.first;
    auto& jointInfo = joint.second;
    if (jointInfo.type_ != JointType::fixed) {
      if (verbose) {
        std::cout << "Rotating joint '" << jointName << "' to have its rotation axis in z-direction" << std::endl;
      }

      const Eigen::Matrix3d R_Jold_W = jointInfo.poseInWorld_.rotationWorldToFrame_.toRotationMatrix();
      const Eigen::Matrix3d R_W_Jold = R_Jold_W.transpose();
      Eigen::Vector3d axisInWorld = R_W_Jold * jointInfo.axisInJointFrame_;
      // Constraint: The new Joint orientation should result in the same axis in world
      // find R_W_Jnew, s.t. axisInWorld != R_W_Jnew * desiredAxisInJointFrame
      const Eigen::Quaterniond q_W_Jnew = Eigen::Quaterniond::FromTwoVectors(desiredAxisInJointFrame, axisInWorld);
      const Eigen::Matrix3d R_W_Jnew = q_W_Jnew.toRotationMatrix();
      assert(axisInWorld.isApprox(R_W_Jnew * desiredAxisInJointFrame));
      jointInfo.poseInWorld_.rotationWorldToFrame_ = q_W_Jnew.inverse();
      jointInfo.axisInJointFrame_ = desiredAxisInJointFrame;
    }
  }
}

bool isFloatinBaseSystem(const urdf::Model& urdf) {
  if (urdf.root_link_->name == "world") {
    if (urdf.getRoot()->child_links.size() != 1) {
      throw std::runtime_error("The world link should have exactly one child link.");
    }
    return false;
  } else {
    return true;
  }
}

void addLinkComAsFrames(UrdfStructure& urdfStructure) {
  for (const auto& link : urdfStructure.links_) {
    FrameInfo comFrameInfo;
    comFrameInfo.parentLinkName_ = link.first;
    comFrameInfo.poseInWorld_ = link.second.inertia_->comFramePoseInWorld_;
    urdfStructure.frames_.insert({link.first + "_COM", comFrameInfo});
  }
}

void sortLinksRecursive(const urdf::LinkSharedPtr& link, const std::map<std::string, unsigned int>& linkId) {
  std::sort(link->child_links.begin(), link->child_links.end(),
            [&linkId](urdf::LinkSharedPtr& a, urdf::LinkSharedPtr& b) { return linkId.at(a->name) < linkId.at(b->name); });
  std::for_each(link->child_links.rbegin(), link->child_links.rend(),
                [&linkId](urdf::LinkSharedPtr& childLink) { sortLinksRecursive(childLink, linkId); });
}

std::map<std::string, unsigned int> getLinkIdMap(const TiXmlDocument& urdfXml) {
  const auto urdf_xml_root = urdfXml.RootElement();

  std::map<std::string, unsigned int> linkIdMap;
  for (auto child = urdf_xml_root->FirstChildElement("link"); child; child = child->NextSiblingElement("link")) {
    std::string linkName;
    if (child->QueryStringAttribute("name", &linkName) == TIXML_SUCCESS) {
      linkIdMap.insert(std::pair<std::string, unsigned int>(linkName, linkIdMap.size() + 1));
    }
  }
  return linkIdMap;
}

void urdf2RobCoGen(const std::string& robotName, urdf::Model urdf, TiXmlDocument urdfXml, const std::string& outputFolder, bool verbose) {
  auto linkId = getLinkIdMap(urdfXml);
  sortLinksRecursive(urdf.root_link_, linkId);

  UrdfStructure urdfStructure;
  urdfStructure.robotName_ = robotName;
  urdfStructure.rootLinkName_ = urdf.root_link_->name;
  urdfStructure.floatingBaseSystem_ = isFloatinBaseSystem(urdf);

  fillUrdfStructureRecursive(*urdf.root_link_, urdfStructure);
  const auto inertiaBeforeAdaptations = getFullInertiaInWorld(urdfStructure);

  if (verbose) {
    std::cout << "\nPrinting Urdf Structure after parsing urdf" << std::endl;
    printLinkJointStructure(urdfStructure);
  }

  // Adapt to fit RobCoGen conventions
  removeFixedJoints(urdfStructure, verbose);
  rotateJointAxesToZaxis(urdfStructure, verbose);
  addLinkComAsFrames(urdfStructure);
  const auto inertiaAfterAdaptations = getFullInertiaInWorld(urdfStructure);

  if (verbose) {
    std::cout << "\nPrinting Urdf Structure after adaptation to RobCoGen conventions" << std::endl;
    printLinkJointStructure(urdfStructure);
  }

  // Check if total inertia is preserved.
  assert(std::abs(inertiaBeforeAdaptations.m_ - inertiaAfterAdaptations.m_) < 1e-6);
  assert(inertiaBeforeAdaptations.I_.isApprox(inertiaAfterAdaptations.I_, 1e-6));

  // Print Kindsl
  const auto prefix = (outputFolder.empty()) ? "" : (outputFolder + "/");
  std::ofstream kindsl_file(prefix + urdfStructure.robotName_ + ".kindsl");
  kindsl_file << generateKindsl(urdfStructure);

  // Print Dtdsl
  std::ofstream dtdsl_file(prefix + urdfStructure.robotName_ + ".dtdsl");
  dtdsl_file << generateDtDsl(urdfStructure);
}