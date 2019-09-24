//
// Created by rgrandia on 23.09.19.
//

#include "urdf2robcogen/ParseFixedFrames.hpp"

#include "urdf2robcogen/utils.hpp"

std::map<std::string, urdf::Joint> parseFixedFrames(const urdf::Model& urdf, const UrdfInfo& urdfInfo) {
  ROS_DEBUG("Parsing Fixed Frames");
  // Fixed frames that will be added to the kindsl file
  std::map<std::string, urdf::Joint> fixedFrames;

  // Move Inertia from fixed frame to closest parent
  moveInertiaFromFixedLinksRecursive(urdf.root_link_);

  // Assign all fixed frames to their closes parent, collect corresponding transforms in "fixedFrames"
  std::for_each(
      urdfInfo.robotFixedFrameNames_.begin(), urdfInfo.robotFixedFrameNames_.end(), [&urdf, &fixedFrames](const std::string& frameName) {
        attachFrameToLink(frameName, fixedFrames, urdf.links_.at(frameName), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
      });

  // for each link, add com as a frame
  for (auto linkName : urdfInfo.robotLinkNames_) {
    addComFrame(urdf, linkName, fixedFrames);
  }

  return fixedFrames;
}

void moveInertiaFromFixedLinksRecursive(const urdf::LinkSharedPtr& link) {
  std::for_each(link->child_links.rbegin(), link->child_links.rend(),
                [](urdf::LinkSharedPtr& childLink) { moveInertiaFromFixedLinksRecursive(childLink); });

  ROS_DEBUG_STREAM("[moveInertiaFromFixedLinksRecursive] -- operating on link: " << link->name);
  if (link->parent_joint && link->parent_joint->type == urdf::Joint::FIXED) {
    if (!link->inertial) {
      return;  // nothing to do if this link does not have inertia
    }
    ROS_DEBUG_STREAM("Transferring inertia to parent " << link->getParent()->name);
    // note: the link from which inertia is to be moved away is referred to as
    // 'child'

    auto parent_link = link->getParent();
    if (!parent_link->inertial) {  // if parent does not have inertial property, need to create it
      parent_link->inertial.reset(new urdf::Inertial());
    }

    moveInertiaFromChildToParent(link, parent_link);
  }
}

void moveInertiaFromChildToParent(const urdf::LinkSharedPtr& child, const urdf::LinkSharedPtr& parent) {
  // (constant because joint is of type FIXED)
  const Eigen::Vector3d P_r_P_C =
      toEigen(child->parent_joint->parent_to_joint_origin_transform.position);  // vector from parent frame to child frame in parent frame
  const Eigen::Quaterniond q_P_C =
      toEigen(child->parent_joint->parent_to_joint_origin_transform.rotation);  // rotation quaternion from child to parent frame

  const Eigen::Vector3d P_r_P_Pcom = toEigen(parent->inertial->origin.position);  // vector from parent frame to parent com in parent frame
  const Eigen::Quaterniond q_P_Pcom =
      toEigen(parent->inertial->origin.rotation);  // rotation quaternion from parent inertia frame to parent frame

  const Eigen::Vector3d C_r_C_Ccom = toEigen(child->inertial->origin.position);  // vector from child frame to child com in child frame
  const Eigen::Quaterniond q_C_Ccom =
      toEigen(child->inertial->origin.rotation);  // rotation quaternion from child inertia frame to child frame

  // rotation matrices
  const Eigen::Matrix3d R_C_Ccom = q_C_Ccom.toRotationMatrix();
  const Eigen::Matrix3d R_P_Pcom = q_P_Pcom.toRotationMatrix();
  const Eigen::Matrix3d R_P_C = q_P_C.toRotationMatrix();

  // rotate the inertia tensors into the respective link frame
  const Eigen::Matrix3d P_I_P_Pcom = R_P_Pcom * inertiaMatrixFromLink(parent) * R_P_Pcom.transpose();
  const Eigen::Matrix3d C_I_C_Ccom = R_C_Ccom * inertiaMatrixFromLink(child) * R_C_Ccom.transpose();

  // find new combined CoM position
  const double m_P = parent->inertial->mass;
  const double m_C = child->inertial->mass;
  const double m_tot = m_P + m_C;

  ROS_DEBUG_STREAM("new total mass " << m_tot);
  ROS_DEBUG_STREAM("of which child mass is " << m_C);
  ROS_DEBUG_STREAM("C_I_C_Ccom\n" << C_I_C_Ccom);

  const Eigen::Vector3d P_r_Pcom_P = -P_r_P_Pcom;
  const Eigen::Vector3d P_r_C_Ccom = R_P_C * C_r_C_Ccom;
  const Eigen::Vector3d P_r_Pcom_Ccom = P_r_Pcom_P + P_r_P_C + P_r_C_Ccom;
  const Eigen::Vector3d P_r_Pcom_totcom = m_C * P_r_Pcom_Ccom / m_tot;
  const Eigen::Vector3d P_r_totcom_Pcom = -P_r_Pcom_totcom;

  const Eigen::Vector3d C_r_totcom_Ccom = R_P_C.transpose() * (P_r_totcom_Pcom + P_r_Pcom_Ccom);

  // calculate inertias in new reference point
  const Eigen::Matrix3d P_I_P_totcom =
      P_I_P_Pcom + m_P * skewSymMatrixFromVector(P_r_totcom_Pcom) * skewSymMatrixFromVector(P_r_totcom_Pcom).transpose();
  const Eigen::Matrix3d child_steiner_term =
      m_C * skewSymMatrixFromVector(C_r_totcom_Ccom) * skewSymMatrixFromVector(C_r_totcom_Ccom).transpose();
  const Eigen::Matrix3d C_I_C_totcom = C_I_C_Ccom + child_steiner_term;

  ROS_DEBUG_STREAM("shift C_r_totcom_Ccom " << C_r_totcom_Ccom.transpose());
  ROS_DEBUG_STREAM("Steiner term contribution\n" << child_steiner_term);
  ROS_DEBUG_STREAM("C_I_C_totcom\n" << C_I_C_totcom);

  // rotate child inertia into parent frame
  const Eigen::Matrix3d P_I_C_totcom = R_P_C * C_I_C_totcom * R_P_C.transpose();

  // sum inertias of two bodies (same frame, same reference point)
  const Eigen::Matrix3d P_I_tot_totcom = P_I_P_totcom + P_I_C_totcom;

  /*
   * assign new inertia properties to parent
   */
  const Eigen::Vector3d P_r_P_totcom = P_r_P_Pcom + P_r_Pcom_totcom;

  parent->inertial->mass = m_tot;
  parent->inertial->origin.position = fromEigen(P_r_P_totcom);
  parent->inertial->origin.rotation.clear();  // new inertia is already in parent frame
  assignInertiaToLink(parent, P_I_tot_totcom);

  // finally, set mass and inertia of child to zero so we know it has been moved
  // away
  child->inertial.reset();
}

void attachFrameToLink(std::string frameName, std::map<std::string, urdf::Joint>& frames, const urdf::LinkConstSharedPtr& link,
                       Eigen::Quaterniond q, Eigen::Vector3d r) {
  if (link->parent_joint->type != urdf::Joint::FIXED) {
    std::string msg = "attachFrameToLink called, but provided child link does not appear to be fixed.";
    throw std::runtime_error(msg);
  }
  if (link->inertial) {
    std::string msg = "attachFrameToLink called, but provided child link still carries inertia.";
    throw std::runtime_error(msg);
  }

  // extract transform between this link (child) and parent and increment input
  // values r corresponds to parent_r_parent_frame position vector q corresponds
  // to q_parent_frame quaternion
  const Eigen::Quaterniond q_parent_child = toEigen(link->parent_joint->parent_to_joint_origin_transform.rotation);
  const Eigen::Vector3d parent_r_parent_child = toEigen(link->parent_joint->parent_to_joint_origin_transform.position);

  q = q_parent_child * q;
  r = parent_r_parent_child + q_parent_child * r;

  if (!link->getParent()->parent_joint or  // clang-format off
      link->getParent()->parent_joint->type == urdf::Joint::CONTINUOUS or
      link->getParent()->parent_joint->type == urdf::Joint::REVOLUTE or
      link->getParent()->parent_joint->type == urdf::Joint::PRISMATIC or
      link->getParent()->parent_joint->type == urdf::Joint::FLOATING) {  // clang-format on
    // attach frame to the parent, which appears to be a "true" link
    urdf::Joint single_frame;
    single_frame.name = frameName;
    single_frame.parent_link_name = link->getParent()->name;
    single_frame.parent_to_joint_origin_transform.position = fromEigen(r);
    single_frame.parent_to_joint_origin_transform.rotation = fromEigen(q);
    frames[frameName] = single_frame;

  } else if (link->getParent()->parent_joint->type == urdf::Joint::FIXED) {
    // the frame must not be attached to a fixed link (which will itself become
    // a frame) hence, call this function recursively until we find a parent
    // that is a true link
    attachFrameToLink(frameName, frames, link->getParent(), q, r);
  } else {
    std::string msg = "ERROR in attachFrameToLink: Unhandled joint type " + std::to_string(link->parent_joint->type);
    throw std::runtime_error(msg);
  }
}

void addComFrame(const urdf::Model& urdf, const std::string& linkName, std::map<std::string, urdf::Joint>& frames) {
  auto link = urdf.links_.at(linkName);
  if (link->inertial == nullptr) {
    std::string msg = "Link " + linkName + " has no inertia.";
    throw std::runtime_error(msg);
  }
  assert(link->inertial);

  urdf::Joint single_frame;
  single_frame.name = linkName + "_COM";
  single_frame.parent_link_name = linkName;
  single_frame.parent_to_joint_origin_transform.clear();
  single_frame.parent_to_joint_origin_transform.position = link->inertial->origin.position;
  single_frame.parent_to_joint_origin_transform.rotation = link->inertial->origin.rotation;
  frames[single_frame.name] = single_frame;
}
