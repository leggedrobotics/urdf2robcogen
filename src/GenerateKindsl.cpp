//
// Created by rgrandia on 25.09.19.
//

#include "urdf2robcogen/GenerateKindsl.hpp"

#include "urdf2robcogen/Utils.hpp"

#include <sstream>

std::string print_inertia_params(const std::string& linkName, const LinkInfo& link) {
  std::ostringstream out, mass, xx, yy, zz, xy, xz, yz;
  out.precision(std::numeric_limits<double>::digits10);
  mass.precision(std::numeric_limits<double>::digits10);
  xx.precision(std::numeric_limits<double>::digits10);
  yy.precision(std::numeric_limits<double>::digits10);
  zz.precision(std::numeric_limits<double>::digits10);
  xy.precision(std::numeric_limits<double>::digits10);
  yz.precision(std::numeric_limits<double>::digits10);
  xz.precision(std::numeric_limits<double>::digits10);

  out << "\tinertia_params {" << std::endl;

  if (!link.inertia_) {
    std::cout << "WARNING: Missing inertia parameters for link " << linkName << std::endl;
    out << "\t}" << std::endl;
    return out.str();
  }

  mass << std::fixed << link.inertia_->m_;
  xx << std::fixed << link.inertia_->I_(0, 0);
  yy << std::fixed << link.inertia_->I_(1, 1);
  zz << std::fixed << link.inertia_->I_(2, 2);
  xy << std::fixed << -link.inertia_->I_(0, 1);
  xz << std::fixed << -link.inertia_->I_(0, 2);
  yz << std::fixed << -link.inertia_->I_(1, 2);

  out << "\t\tmass = " << mass.str() << std::endl
      << "\t\tCoM = "
      << "(0.0, 0.0, 0.0)" << std::endl
      << "\t\tIx = " << std::fixed << xx.str() << std::endl
      << "\t\tIy = " << std::fixed << yy.str() << std::endl
      << "\t\tIz = " << std::fixed << zz.str() << std::endl
      << "\t\tIxy = " << std::fixed << xy.str() << std::endl
      << "\t\tIxz = " << std::fixed << xz.str() << std::endl
      << "\t\tIyz = " << std::fixed << yz.str() << std::endl
      << "\t\tref_frame = "
      << "fr_" << linkName << "_COM" << std::endl
      << "\t}" << std::endl;

  return out.str();
}

std::string print_children(const std::string& linkName, const LinkInfo& link, const std::map<std::string, LinkInfo>& links) {
  std::ostringstream out;
  bool style = false;
  out << "\tchildren {";
  for (const auto childName : link.childLinks_) {
    if (!style) {
      out << std::endl;
      style = true;
    }
    out << "\t"
        << "\t" << childName << " via " << links.at(childName).parentJoint_ << std::endl;
  }
  if (style) out << "\t";
  out << "}" << std::endl;
  return out.str();
}

std::string print_translation(const Eigen::Vector3d translation) {
  std::ostringstream out;
  // the position of the joint expressed in the parent_link frame
  out << "translation = " << printVector(fromEigen(translation)) << std::endl;
  return out.str();
}

std::string print_rotation(const Eigen::Quaterniond& rotation) {
  std::ostringstream out;
  // the orientation of the joint expressed in the parent_link frame
  out << "rotation = " << printVector(rotationXyz(fromEigen(rotation))) << std::endl;
  return out.str();
}

std::string print_frame(const std::string& linkName, const LinkInfo& link, const UrdfStructure& urdfStructure) {
  std::ostringstream out;
  bool print_title = false;
  for (const auto& f : urdfStructure.frames_) {
    if (f.second.parentLinkName_ == linkName) {
      if (!print_title) {
        out << "\tframes {" << std::endl;
        print_title = true;
      }
      PoseInWorld parentPoseInWorld;
      if (linkName == urdfStructure.rootLinkName_) {
        parentPoseInWorld.position_ = Eigen::Vector3d::Zero();
        parentPoseInWorld.rotationWorldToFrame_ = Eigen::Quaterniond::Identity();
      } else {
        parentPoseInWorld = urdfStructure.joints_.at(link.parentJoint_).poseInWorld_;
      }
      auto relativePose = getRelativePose(f.second.poseInWorld_, parentPoseInWorld);
      out << "\t\tfr_" << f.first << " {" << std::endl;
      out << "\t\t\t" << print_translation(relativePose.first);
      out << "\t\t\t" << print_rotation(relativePose.second);
      out << "\t\t}" << std::endl;
    }
  }
  if (print_title) {
    out << "\t}" << std::endl;
  }

  return out.str();
}

std::string generateKindsl(const UrdfStructure& urdfStructure) {
  std::ostringstream out;

  // Base
  {
    out << print_timeStamp();
    out << "/*\n * Robot base\n */" << std::endl;
    out << "Robot " << urdfStructure.robotName_ << " {" << std::endl;
    out << "RobotBase " << urdfStructure.rootLinkName_;
    if (urdfStructure.floatingBaseSystem_) {
      out << " floating";
    }
    out << " {" << std::endl;
    out << print_inertia_params(urdfStructure.rootLinkName_, urdfStructure.links_.at(urdfStructure.rootLinkName_));
    out << print_children(urdfStructure.rootLinkName_, urdfStructure.links_.at(urdfStructure.rootLinkName_), urdfStructure.links_);
    out << print_frame(urdfStructure.rootLinkName_, urdfStructure.links_.at(urdfStructure.rootLinkName_), urdfStructure);
    out << "}" << std::endl;
  }

  // links
  out << "\n/*\n * Links\n */" << std::endl;
  unsigned int robcogen_link_id = 1;
  for (const auto& link : toIDSortedVector(urdfStructure.links_)) {
    const auto& linkName = link.first;
    if (linkName != urdfStructure.rootLinkName_) {
      out << std::endl;
      out << "link " << linkName << " {" << std::endl;
      out << "\tid = " << robcogen_link_id++ << std::endl;
      out << print_inertia_params(linkName, link.second);
      out << print_children(linkName, link.second, urdfStructure.links_);
      out << print_frame(linkName, link.second, urdfStructure);
      out << "}" << std::endl;
    }
  }

  // joints
  out << "\n/*\n * Joints\n */" << std::endl;
  for (const auto& joint : toIDSortedVector(urdfStructure.joints_)) {
    const auto& jointName = joint.first;
    const auto& jointInfo = joint.second;
    PoseInWorld parentPoseInWorld;
    if (jointInfo.parentLink_ == urdfStructure.rootLinkName_) {
      parentPoseInWorld.position_ = Eigen::Vector3d::Zero();
      parentPoseInWorld.rotationWorldToFrame_ = Eigen::Quaterniond::Identity();
    } else {
      const auto& parentLink = urdfStructure.links_.at(jointInfo.parentLink_);
      parentPoseInWorld = urdfStructure.joints_.at(parentLink.parentJoint_).poseInWorld_;
    }
    auto relativePose = getRelativePose(jointInfo.poseInWorld_, parentPoseInWorld);

    out << std::endl;
    out << toString(jointInfo.type_) + " ";
    out << jointName << " {" << std::endl;
    out << "\tref_frame {" << std::endl;
    out << "\t\t" << print_translation(relativePose.first);
    out << "\t\t" << print_rotation(relativePose.second);
    out << "\t}" << std::endl;
    out << "}" << std::endl;
  }

  // end of robot
  out << "\n}" << std::endl;

  return out.str();
}