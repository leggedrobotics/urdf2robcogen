//
// Created by rgrandia on 23.09.19.
//

#include "urdf2robcogen/generateKindsl.hpp"

#include "urdf2robcogen/utils.hpp"

#include <fstream>

std::string print_inertia_params(const urdf::Link& link) {
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

  if (!link.inertial) {
    std::cout << "WARNING: Missing inertia parameters for link " << link.name << std::endl;
    out << "\t}" << std::endl;
    return out.str();
  }

  mass << std::fixed << link.inertial->mass;
  xx << std::fixed << link.inertial->ixx;
  yy << std::fixed << link.inertial->iyy;
  zz << std::fixed << link.inertial->izz;
  xy << std::fixed << -link.inertial->ixy;
  xz << std::fixed << -link.inertial->ixz;
  yz << std::fixed << -link.inertial->iyz;

  out << "\t\tmass = " << mass.str() << std::endl
      << "\t\tCoM = " << "(0.0, 0.0, 0.0)" << std::endl
      << "\t\tIx = " << std::fixed << xx.str() << std::endl
      << "\t\tIy = " << std::fixed << yy.str() << std::endl
      << "\t\tIz = " << std::fixed << zz.str() << std::endl
      << "\t\tIxy = " << std::fixed << xy.str() << std::endl
      << "\t\tIxz = " << std::fixed << xz.str() << std::endl
      << "\t\tIyz = " << std::fixed << yz.str() << std::endl
      << "\t\tref_frame = "
      << "fr_" << link.name << "_COM" << std::endl
      << "\t}" << std::endl;

  return out.str();
}

std::string print_children(const std::vector<std::string>& children, const std::map<std::string, std::string>& robotLinkJointParentNames) {
  std::ostringstream out;
  bool style = false;
  out << "\tchildren {";
  for (const auto childName : children) {
    if (!style) {
      out << std::endl;
      style = true;
    }
    out << "\t"
        << "\t" << childName << " via " << robotLinkJointParentNames.at(childName) << std::endl;
  }
  if (style) out << "\t";
  out << "}" << std::endl;
  return out.str();
}

std::string print_translation(const urdf::Joint& joint) {
  std::ostringstream out;
  // the position of the joint expressed in the parent_link frame
  out << "translation = " << printVector(joint.parent_to_joint_origin_transform.position) << std::endl;
  return out.str();
}

std::string print_rotation(const urdf::Joint& joint) {
  std::ostringstream out;
  // the orientation of the joint expressed in the parent_link frame
  out << "rotation = " << printVector(rotationXyz(joint.parent_to_joint_origin_transform.rotation)) << std::endl;
  return out.str();
}

std::string print_frame(const std::map<std::string, urdf::Joint>& fixedFrames, const urdf::Link& link) {
  std::ostringstream out;
  bool print_title = false;
  for (const auto& f : fixedFrames) {
    if (f.second.parent_link_name == link.name) {
      if (!print_title) {
        out << "\tframes {" << std::endl;
        print_title = true;
      }
      out << "\t\tfr_" << f.second.name << " {" << std::endl;
      out << "\t\t\t" << print_translation(f.second);
      out << "\t\t\t" << print_rotation(f.second);
      out << "\t\t}" << std::endl;
    }
  }
  if (print_title) {
    out << "\t}" << std::endl;
  }

  return out.str();
}

void generateKindsl(const std::string& robotName, const urdf::Model& urdf, const UrdfInfo& urdfInfo, const std::map<std::string, urdf::Joint>& fixedFrames){
  const auto& robotJointParentNames_ = urdfInfo.robotJointParentNames_;
  const auto& robotLinkJointParentNames_ = urdfInfo.robotLinkJointParentNames_;
  const auto& robotLinkChildrenNames_ = urdfInfo.robotLinkChildrenNames_;

  // reverse the link and joint vectors -> our parsing produced the inverse order
  auto robotLinkNames = urdfInfo.robotLinkNames_;
  auto robotJointNames = urdfInfo.robotJointNames_;
  std::reverse(robotLinkNames.begin(), robotLinkNames.end());
  std::reverse(robotJointNames.begin(), robotJointNames.end());


  std::string kindsl_name = robotName + ".kindsl";
  std::ofstream kindsl_file(kindsl_name.c_str());

  print_timeStamp(kindsl_file);

  // Base
  {
    auto linkChildren = robotLinkChildrenNames_.at(urdfInfo.rootLinkName_);
    std::reverse(linkChildren.begin(), linkChildren.end());

    kindsl_file << "/*\n * Robot base\n */" << std::endl;
    kindsl_file << "Robot " << robotName << " {" << std::endl;
    kindsl_file << "RobotBase " << urdfInfo.rootLinkName_;
    if (urdfInfo.floatingBaseSystem_) {
      kindsl_file << " floating";
    }
    kindsl_file << " {" << std::endl;
    kindsl_file << print_inertia_params(*urdf.root_link_);
    kindsl_file << print_children(linkChildren, robotLinkJointParentNames_);
    kindsl_file << print_frame(fixedFrames, *urdf.root_link_);
    kindsl_file << "}" << std::endl;
  }

  // links
  kindsl_file << "\n/*\n * Links\n */" << std::endl;
  unsigned int robcogen_link_id = 1;
  for (auto l_name : robotLinkNames) {
    if (l_name != urdfInfo.rootLinkName_) {
      auto linkChildren = robotLinkChildrenNames_.at(l_name);
      std::reverse(linkChildren.begin(), linkChildren.end());

      kindsl_file << std::endl;
      kindsl_file << "link " << l_name << " {" << std::endl;
      kindsl_file << "\tid = " << robcogen_link_id++ << std::endl;
      ROS_DEBUG_STREAM("printing inertia params for link " << l_name);
      ROS_DEBUG_STREAM(print_inertia_params(*urdf.links_.at(l_name)));
      ROS_DEBUG_STREAM("printing frames for that same link" );
      ROS_DEBUG_STREAM(print_frame(fixedFrames, *urdf.links_.at(l_name)));
      kindsl_file << print_inertia_params(*urdf.links_.at(l_name));
      kindsl_file << print_children(linkChildren, robotLinkJointParentNames_);
      kindsl_file << print_frame(fixedFrames, *urdf.links_.at(l_name));
      kindsl_file << "}" << std::endl;
    }
  }

  // joints
  kindsl_file << "\n/*\n * Joints\n */" << std::endl;

  for (const auto& j_name : robotJointNames) {
    kindsl_file << std::endl;

    switch (urdf.joints_.at(j_name)->type) {
      case urdf::Joint::CONTINUOUS:
      case urdf::Joint::REVOLUTE: {
        kindsl_file << "r_joint ";
        break;
      }
      case urdf::Joint::PRISMATIC: {
        kindsl_file << "p_joint ";
        break;
      }
      default: { throw std::runtime_error("Unhandled joint type when writing kindsl."); }
    }

    kindsl_file << j_name << " {" << std::endl;
    kindsl_file << "\tref_frame {" << std::endl;
    kindsl_file << "\t\t" << print_translation(*urdf.joints_.at(j_name));
    kindsl_file << "\t\t" << print_rotation(*urdf.joints_.at(j_name));
    kindsl_file << "\t}" << std::endl;
    kindsl_file << "}" << std::endl;
  }

  // end of robot
  kindsl_file << "\n}" << std::endl;

  kindsl_file.close();
}