#include <urdf2robcogen/Urdf2RobCoGen.hpp>

#include <Eigen/Eigenvalues>
#include <ctime>
#include <iomanip>
#include <kindr/Core>
#include <stdexcept>

namespace urdf2robcogen {

Urdf2RobCoGen::Urdf2RobCoGen(const std::string& path_to_urdf_file, const std::string& robot_name, bool debug_flag)
    : robot_name_(robot_name), floating_base_(true), debug_mode_(debug_flag) {
  if (!urdf_model_.initFile(path_to_urdf_file)) {
    throw std::runtime_error("Invalid URDF File");
  }
  urdf_xml_.LoadFile(path_to_urdf_file);
}

void Urdf2RobCoGen::parseUrdf() {
  root_link_name_ = urdf_model_.getRoot()->name;

  std::cout << "Parsing the urdf..." << std::endl;
  std::cout << "The root of the urdf model is: " << root_link_name_ << std::endl;

  if (root_link_name_ == "world") {
    // this is a fixed-base system, adjust root values to robot base
    std::cout << "We assume this is a fixed-base system." << std::endl;
    if (urdf_model_.getRoot()->child_links.size() != 1) {
      throw std::runtime_error("The world link should have exactly one child link.");
    }
    // root_link_name_ = urdf_model_.getRoot()->child_links[0]->name;
    // std::cout << "The first robot link is called " << root_link_name_ << std::endl;
    // urdf_xml_root = urdf_xml_.RootElement()->FirstChildElement("link");
    floating_base_ = false;
  }

  const auto urdf_xml_root = urdf_xml_.RootElement();

  if (debug_mode_) {
    std::cout << "Ordered list of URDF links:" << std::endl;
  }

  urdf_link_id_.clear();
  for (auto child = urdf_xml_root->FirstChildElement("link"); child; child = child->NextSiblingElement("link")) {
    std::string linkName;
    if (child->QueryStringAttribute("name", &linkName) == TIXML_SUCCESS) {
      urdf_link_id_.insert(std::pair<std::string, unsigned int>(linkName, urdf_link_id_.size() + 1));
      if (debug_mode_) {
        std::cout << linkName << std::endl;
      }
    }
  }
  if (debug_mode_) {
    std::cout << "--- End of list. ---" << std::endl;
  }

  setLinkNamesFromUrdf();
}

void Urdf2RobCoGen::setLinkNamesFromUrdf() {
  std::cout << "Constructing the link tree from the urdf file... " << std::endl;

  if (robot_link_names_.size()) {
    std::cout << "WARNING link names already exist and are not going to change!" << std::endl;
    std::cout << "press <any key> to continue..." << std::endl;
    std::getchar();
    return;
  }

  parseUrdfTreeRecursive(urdf_model_.root_link_);

  // fix the orphan frames : Add frames to closest parent link
  std::for_each(orphan_frames_.begin(), orphan_frames_.end(),
                [this](const std::string& frameName) { this->attachFrameToLink(frameName, urdf_model_.links_[frameName]); });

  // for each link, add com as a frame
  for (auto linkName : robot_link_names_) {
    auto link = urdf_model_.links_[linkName];
    if (link->inertial == nullptr) {
      std::cout << "Link " << linkName << " has no inertia." << std::endl;
      throw std::runtime_error("Expected inertia.");
    }
    assert(link->inertial);

    urdf::Joint single_frame;
    single_frame.name = linkName + "_COM";
    single_frame.parent_link_name = linkName;
    single_frame.parent_to_joint_origin_transform.clear();
    single_frame.parent_to_joint_origin_transform.position = link->inertial->origin.position;
    single_frame.parent_to_joint_origin_transform.rotation = link->inertial->origin.rotation;
    frames_[single_frame.name] = single_frame;
  }

  if (debug_mode_) {
    std::cout << "Reading Link Names fom URDF" << std::endl;
    std::cout << "Joints names are extracted from the URDF as well" << std::endl;
    printLinkJointNames();
  }
}

void Urdf2RobCoGen::parseUrdfTreeRecursive(const urdf::LinkSharedPtr& link) {
  /*
   * Operate on the kinematic tree from bottom (leaves) to root.
   * This ensures the inertias are all propagated correctly.
   * Additionally, sort the child links according to the order in the URDF.
   */
  std::sort(link->child_links.begin(), link->child_links.end(),
            [this](urdf::LinkSharedPtr& a, urdf::LinkSharedPtr& b) { return this->urdf_link_id_[a->name] < this->urdf_link_id_[b->name]; });
  std::for_each(link->child_links.rbegin(), link->child_links.rend(),
                [this](urdf::LinkSharedPtr& childLink) { this->parseUrdfTreeRecursive(childLink); });

  std::cout << "---------------------------------------------------------------" << std::endl;
  std::cout << "Recursive URDF parsing -- operating on link: " << link->name << std::endl;

  if (!link->parent_joint) {
    if (link->name != urdf_model_.getRoot()->name) {
      std::string msg = "ERROR: Encountered a link without a parent joint before reaching the root.";
      throw std::runtime_error(msg);
    }
    std::cout << "This is the root link. Adding it to the model." << std::endl;
    robot_link_names_.push_back(link->name);

  } else {  // there exists a parent joint
    switch (link->parent_joint->type) {
      case urdf::Joint::FIXED: {
        std::cout << "Parent joint is fixed. Will add this link as a frame." << std::endl;
        std::cout << "Transferring inertia to parent " << link->getParent()->name << std::endl;
        moveInertiaFromFixedLink(link);
        orphan_frames_.push_back(link->name);
        break;
      }
      case urdf::Joint::CONTINUOUS:
      case urdf::Joint::REVOLUTE: {
        std::cout << "Parent joint is revolute. Will add this link and joint " << link->parent_joint->name << " to the model." << std::endl;
        robot_link_names_.push_back(link->name);
        robot_joint_names_.push_back(link->parent_joint->name);
        break;
      }
      case urdf::Joint::PRISMATIC: {
        std::cout << "Parent joint is prismatic. Will add this link and joint " << link->parent_joint->name << " to the model."
                  << std::endl;
        robot_link_names_.push_back(link->name);
        robot_joint_names_.push_back(link->parent_joint->name);
        break;
      }
      case urdf::Joint::FLOATING: {
        if (link->name != urdf_model_.getRoot()->name) {
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

void Urdf2RobCoGen::moveInertiaFromFixedLink(const urdf::LinkSharedPtr& link) {
  // note: the link from which inertia is to be moved away is referred to as
  // 'child'

  if (!link->parent_joint or link->parent_joint->type != urdf::Joint::FIXED) {
    std::string msg = "ERROR: moveInertiaFromFixedLink should not be called for links that don't have a fixed parent joint.";
    throw std::runtime_error(msg);
  }

  if (!link->inertial) {
    return;  // nothing to do if this link does not have inertia
  }

  auto parent_link = link->getParent();

  if (!parent_link->inertial) {
    // if parent does not have inertial property, need to create it
    parent_link->inertial.reset(new urdf::Inertial());
  }
  if (debug_mode_) {
    std::cout << "[moveInertiaFromFixedLink] parent " << parent_link->name << " old mass " << parent_link->inertial->mass << std::endl;
  }

  /*
   * Extract the required positions and orientations
   */

  Eigen::Vector3d P_r_P_Pcom;                                           // vector from parent frame to parent com in parent frame
  P_r_P_Pcom << parent_link->inertial->origin.position.x,  // clang-format off
                parent_link->inertial->origin.position.y,
                parent_link->inertial->origin.position.z; // clang-format-on

  Eigen::Quaterniond q_P_Pcom;  // rotation quaternion from parent inertia frame to parent frame
  parent_link->inertial->origin.rotation.getQuaternion(q_P_Pcom.x(), q_P_Pcom.y(), q_P_Pcom.z(), q_P_Pcom.w());

  Eigen::Vector3d P_r_P_C;  // vector from parent frame to child frame in parent frame
  P_r_P_C << link->parent_joint->parent_to_joint_origin_transform.position.x, // clang-format off
             link->parent_joint->parent_to_joint_origin_transform.position.y,
             link->parent_joint->parent_to_joint_origin_transform.position.z;  // clang-format on

  Eigen::Quaterniond q_P_C;  // rotation quaternion from child to parent frame
                             // (constant because joint is of type FIXED)
  link->parent_joint->parent_to_joint_origin_transform.rotation.getQuaternion(q_P_C.x(), q_P_C.y(), q_P_C.z(), q_P_C.w());

  Eigen::Vector3d C_r_C_Ccom;             // vector from child frame to child com in child frame
  C_r_C_Ccom << link->inertial->origin.position.x,  // clang-format off
                link->inertial->origin.position.y,
                link->inertial->origin.position.z;  // clang-format on

  Eigen::Quaterniond q_C_Ccom;  // rotation quaternion from child inertia frame to child frame
  link->inertial->origin.rotation.getQuaternion(q_C_Ccom.x(), q_C_Ccom.y(), q_C_Ccom.z(), q_C_Ccom.w());

  // rotation matrices
  const Eigen::Matrix3d R_C_Ccom = q_C_Ccom.toRotationMatrix();
  const Eigen::Matrix3d R_P_Pcom = q_P_Pcom.toRotationMatrix();
  const Eigen::Matrix3d R_P_C = q_P_C.toRotationMatrix();

  // rotate the inertia tensors into the respective link frame
  const Eigen::Matrix3d P_I_P_Pcom = R_P_Pcom * inertiaMatrixFromLink(parent_link) * R_P_Pcom.transpose();
  const Eigen::Matrix3d C_I_C_Ccom = R_C_Ccom * inertiaMatrixFromLink(link) * R_C_Ccom.transpose();

  // find new combined CoM position
  const double m_P = parent_link->inertial->mass;
  const double m_C = link->inertial->mass;
  const double m_tot = m_P + m_C;

  if (debug_mode_) {
    std::cout << "new total mass " << m_tot << std::endl;
    std::cout << "of which child mass is " << m_C << std::endl;
    std::cout << "C_I_C_Ccom\n" << C_I_C_Ccom << std::endl;
  }

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

  if (debug_mode_) {
    std::cout << "shift C_r_totcom_Ccom " << C_r_totcom_Ccom.transpose() << std::endl;
    std::cout << "Steiner term contribution\n" << child_steiner_term << std::endl;
    std::cout << "C_I_C_totcom\n" << C_I_C_totcom << std::endl;
  }

  // rotate child inertia into parent frame
  const Eigen::Matrix3d P_I_C_totcom = R_P_C * C_I_C_totcom * R_P_C.transpose();

  // sum inertias of two bodies (same frame, same reference point)
  const Eigen::Matrix3d P_I_tot_totcom = P_I_P_totcom + P_I_C_totcom;

  /*
   * assign new inertia properties to parent
   */
  const Eigen::Vector3d P_r_P_totcom = P_r_P_Pcom + P_r_Pcom_totcom;

  parent_link->inertial->mass = m_tot;
  parent_link->inertial->origin.position = urdf::Vector3(P_r_P_totcom(0), P_r_P_totcom(1), P_r_P_totcom(2));
  parent_link->inertial->origin.rotation.clear();  // new inertia is already in parent frame
  assignInertiaToLink(parent_link, P_I_tot_totcom);

  // finally, set mass and inertia of child to zero so we know it has been moved
  // away
  link->inertial.reset();
}

Eigen::Matrix3d Urdf2RobCoGen::inertiaMatrixFromLink(const urdf::LinkConstSharedPtr& link) {
  Eigen::Matrix3d inertia;

  inertia(0, 0) = link->inertial->ixx;
  inertia(1, 1) = link->inertial->iyy;
  inertia(2, 2) = link->inertial->izz;
  inertia(0, 1) = inertia(1, 0) = link->inertial->ixy;
  inertia(0, 2) = inertia(2, 0) = link->inertial->ixz;
  inertia(1, 2) = inertia(2, 1) = link->inertial->iyz;

  return inertia;
}

void Urdf2RobCoGen::assignInertiaToLink(const urdf::LinkSharedPtr& link, const Eigen::Matrix3d& inertia) {
  // sanity check on inertia
  assert(inertia(0, 1) == inertia(1, 0));
  assert(inertia(0, 2) == inertia(2, 0));
  assert(inertia(1, 2) == inertia(2, 1));

  Eigen::VectorXcd eigenvalues = inertia.eigenvalues();
  assert(eigenvalues.real().minCoeff() > 0.0 and eigenvalues.imag().cwiseAbs().maxCoeff() < 1e-9);

  link->inertial->ixx = inertia(0, 0);
  link->inertial->iyy = inertia(1, 1);
  link->inertial->izz = inertia(2, 2);
  link->inertial->ixy = inertia(0, 1);
  link->inertial->ixz = inertia(0, 2);
  link->inertial->iyz = inertia(1, 2);
}

Eigen::Matrix3d Urdf2RobCoGen::skewSymMatrixFromVector(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d skew;
  skew << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;

  return skew;
}

void Urdf2RobCoGen::attachFrameToLink(std::string frameName, const urdf::LinkConstSharedPtr& link, Eigen::Quaterniond q,
                                      Eigen::Vector3d r) {
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

  Eigen::Quaterniond q_parent_child;
  link->parent_joint->parent_to_joint_origin_transform.rotation.getQuaternion(q_parent_child.x(), q_parent_child.y(), q_parent_child.z(),
                                                                              q_parent_child.w());
  q = q_parent_child * q;

  Eigen::Vector3d parent_r_parent_child;
  parent_r_parent_child << link->parent_joint->parent_to_joint_origin_transform.position.x,  // clang-format off
                           link->parent_joint->parent_to_joint_origin_transform.position.y,
                           link->parent_joint->parent_to_joint_origin_transform.position.z;  // clang-format on

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
    single_frame.parent_to_joint_origin_transform.position.clear();
    single_frame.parent_to_joint_origin_transform.position.x = r(0);
    single_frame.parent_to_joint_origin_transform.position.y = r(1);
    single_frame.parent_to_joint_origin_transform.position.z = r(2);

    single_frame.parent_to_joint_origin_transform.rotation.setFromQuaternion(q.x(), q.y(), q.z(), q.w());
    frames_[frameName] = single_frame;

  } else if (link->getParent()->parent_joint->type == urdf::Joint::FIXED) {
    // the frame must not be attached to a fixed link (which will itself become
    // a frame) hence, call this function recursively until we find a parent
    // that is a true link
    attachFrameToLink(frameName, link->getParent(), q, r);
  } else {
    std::cout << "ERROR in attachFrameToLink: Unhandled joint type " << link->parent_joint->type << std::endl;
    throw std::runtime_error("Unhandled joint type");
  }
}

void Urdf2RobCoGen::fixJointFrameRecursive(urdf::LinkSharedPtr& link, const Eigen::Quaterniond q_old_new) {
  /**
   * q_old_new is the rotation that was already applied to the parent joint
   * of this link. We must therefore first update the inertia properties.
   */
  if (!link) {
    throw std::runtime_error("link pointer not valid");
  }

  /**
   * for each (true) child joint, update its orientation such that
   * the axis points in the new z direction (convention required by Robcogen)
   * Then, call this function for the corresponding child link
   */
  for (auto child_joint : link->child_joints) {
    // verify it is a true joint and not a frame
    if (std::find(robot_joint_names_.begin(), robot_joint_names_.end(), child_joint->name) == robot_joint_names_.end()) {
      // apply recursively to child frames (which may have movable children, i.e. true links)
      fixJointFrameRecursive(urdf_model_.links_[child_joint->child_link_name], q_old_new);
      continue;  // we will handle frames later, no need to rotate them here
    }

    std::cout << "[fixJointFrameRecursive] Processing joint " << child_joint->name << std::endl;

    // update position of child joint in link frame to new link orientation
    Eigen::Vector3d old_r_childjoint;
    old_r_childjoint << child_joint->parent_to_joint_origin_transform.position.x,  // clang-format off
                        child_joint->parent_to_joint_origin_transform.position.y,
                        child_joint->parent_to_joint_origin_transform.position.z;  // clang-format on
    Eigen::Vector3d new_r_childjoint = q_old_new.inverse() * old_r_childjoint;
    child_joint->parent_to_joint_origin_transform.position.x = new_r_childjoint(0);
    child_joint->parent_to_joint_origin_transform.position.y = new_r_childjoint(1);
    child_joint->parent_to_joint_origin_transform.position.z = new_r_childjoint(2);

    // find new orientation of child joint such that its axis is aligned with
    // the z axis of the child frame
    Eigen::Quaterniond q_old_oldChild;  // rotation from old parent frame to
                                        // original child frame
    child_joint->parent_to_joint_origin_transform.rotation.getQuaternion(q_old_oldChild.x(), q_old_oldChild.y(), q_old_oldChild.z(),
                                                                         q_old_oldChild.w());

    Eigen::Vector3d oldChild_axis;  // the original rotation axis in joint(=child) frame
    oldChild_axis << child_joint->axis.x, child_joint->axis.y, child_joint->axis.z;

    // define rotation how old child frame must be rotated such that axis aligns
    // with z
    Eigen::Quaterniond q_oldChild_newChild = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), oldChild_axis);

    // new orientation of the child frame in the new parent frame
    Eigen::Quaterniond q_new_newChild = q_old_new.inverse() * q_old_oldChild * q_oldChild_newChild;

    child_joint->parent_to_joint_origin_transform.rotation.setFromQuaternion(q_new_newChild.x(), q_new_newChild.y(), q_new_newChild.z(),
                                                                             q_new_newChild.w());

    // finally, perform update recursively on the child link
    fixJointFrameRecursive(urdf_model_.links_[child_joint->child_link_name], q_oldChild_newChild);
  }

  /**
   * for each frame whose parent is this link, update its position and
   * orientation according to q_old_new
   */
  for (auto& frame : frames_) {
    auto& joint = frame.second;

    if (joint.parent_link_name != link->name) {
      continue;  // not interested in frames that are attached elsewhere
    }

    // update position of frame in link frame to new link orientation
    Eigen::Vector3d old_r_frame;
    old_r_frame << joint.parent_to_joint_origin_transform.position.x,  // clang-format off
                   joint.parent_to_joint_origin_transform.position.y,
                   joint.parent_to_joint_origin_transform.position.z;  // clang-format on
    Eigen::Vector3d new_r_frame = q_old_new.inverse() * old_r_frame;
    joint.parent_to_joint_origin_transform.position.x = new_r_frame(0);
    joint.parent_to_joint_origin_transform.position.y = new_r_frame(1);
    joint.parent_to_joint_origin_transform.position.z = new_r_frame(2);

    // find new orientation of frame
    Eigen::Quaterniond q_old_frame;  // rotation from old parent frame to original child frame
    joint.parent_to_joint_origin_transform.rotation.getQuaternion(q_old_frame.x(), q_old_frame.y(), q_old_frame.z(), q_old_frame.w());
    Eigen::Quaterniond q_new_frame = q_old_new.inverse() * q_old_frame;
    joint.parent_to_joint_origin_transform.rotation.setFromQuaternion(q_new_frame.x(), q_new_frame.y(), q_new_frame.z(), q_new_frame.w());
  }
}

void Urdf2RobCoGen::setAdditionalFrames(const std::vector<std::string>& frames, const std::vector<std::string>& parents,
                                        const std::vector<urdf::Vector3>& translations, const std::vector<urdf::Vector3>& rotations) {
  if (!robot_link_names_.size()) {
    throw std::runtime_error("Link names must already be filled before setting additional frames.");
  }

  // frames, parents, translations and rotations should has same size
  const auto nFrames = frames.size();
  if (nFrames != parents.size() || nFrames != translations.size() || nFrames != rotations.size()) {
    throw std::runtime_error("frames, parents, translation and rotation vectors should have the same size");
  }

  // add the frames to the model
  for (size_t k = 0; k < frames.size(); ++k) {
    auto parent_link_name = parents.at(k);

    if (urdf_model_.links_.find(parent_link_name) == urdf_model_.links_.end()) {
      std::cout << "ERROR: when adding additional frame " << frames.at(k) << ", could not find parent link " << parent_link_name
                << " in URDF model." << std::endl;
      throw std::runtime_error("Frame without parent");
    }

    Eigen::Vector3d r(translations.at(k).x, translations.at(k).y, translations.at(k).z);
    Eigen::Quaterniond q = Eigen::AngleAxisd(rotations.at(k).x, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(rotations.at(k).y, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(rotations.at(k).z, Eigen::Vector3d::UnitZ());

    attachFrameToLink(frames.at(k), urdf_model_.links_[parents.at(k)], q, r);
  }

  if (debug_mode_) {
    std::cout << "User is setting additional frames. List of all frames:" << std::endl;
    for (auto fname : frames_) std::cout << fname.first << std::endl;
    std::cout << "press <any key> to continue..." << std::endl;
    std::getchar();
  }
}

void Urdf2RobCoGen::fixJointParents(const std::string& jointName) {
  auto current_parent_link_name = urdf_model_.joints_[jointName]->parent_link_name;
  const auto movableParentLinkName = getParentLinkName(urdf_model_.links_[urdf_model_.joints_[jointName]->child_link_name]);

  Eigen::Quaterniond q_P_C;  //! quaternion parent child
  Eigen::Vector3d P_r_P_C;   //! position parent child

  urdf_model_.joints_[jointName]->parent_to_joint_origin_transform.rotation.getQuaternion(q_P_C.x(), q_P_C.y(), q_P_C.z(), q_P_C.w());

  P_r_P_C << urdf_model_.joints_[jointName]->parent_to_joint_origin_transform.position.x,  // clang-format off
             urdf_model_.joints_[jointName]->parent_to_joint_origin_transform.position.y,
             urdf_model_.joints_[jointName]->parent_to_joint_origin_transform.position.z;  // clang-format on

  while (current_parent_link_name != movableParentLinkName) {
    auto PJoint = urdf_model_.links_[current_parent_link_name]->parent_joint;

    Eigen::Quaterniond q_Pnext_P;     //! quaternion parent child
    Eigen::Vector3d Pnext_r_Pnext_P;  //! position parent child

    PJoint->parent_to_joint_origin_transform.rotation.getQuaternion(q_Pnext_P.x(), q_Pnext_P.y(), q_Pnext_P.z(), q_Pnext_P.w());
    Pnext_r_Pnext_P << PJoint->parent_to_joint_origin_transform.position.x,  // clang-format off
                       PJoint->parent_to_joint_origin_transform.position.y,
                       PJoint->parent_to_joint_origin_transform.position.z;  // clang-format on

    q_P_C = q_Pnext_P * q_P_C;                        // propagate rotation
    P_r_P_C = q_Pnext_P * P_r_P_C + Pnext_r_Pnext_P;  // propagate position

    current_parent_link_name = urdf_model_.links_[current_parent_link_name]->getParent()->name;  // step up the tree
  }

  // finally, assign to the joint again
  urdf_model_.joints_[jointName]->parent_to_joint_origin_transform.rotation.setFromQuaternion(q_P_C.x(), q_P_C.y(), q_P_C.z(), q_P_C.w());
  urdf_model_.joints_[jointName]->parent_to_joint_origin_transform.position.x = P_r_P_C(0);
  urdf_model_.joints_[jointName]->parent_to_joint_origin_transform.position.y = P_r_P_C(1);
  urdf_model_.joints_[jointName]->parent_to_joint_origin_transform.position.z = P_r_P_C(2);
}

bool Urdf2RobCoGen::generateFiles() {
  bool f1 = generateKindsl();
  bool f2 = generateDtDsl();
  return (f1 && f2);
}

bool Urdf2RobCoGen::generateKindsl() {
  if (robot_link_names_.empty()) {
    throw std::runtime_error("Link names are still empty.");
  }

  // fix frames!
  fixJointFrameRecursive(urdf_model_.root_link_);

  // fix joint parents
  std::for_each(robot_joint_names_.begin(), robot_joint_names_.end(), [this](const std::string& jointName) { fixJointParents(jointName); });

  // reverse the link and joint vectors -> our parsing produced the inverse
  // order
  std::reverse(robot_link_names_.begin(), robot_link_names_.end());
  std::reverse(robot_joint_names_.begin(), robot_joint_names_.end());

  std::string kindsl_name = robot_name_ + ".kindsl";
  std::ofstream kindsl_file(kindsl_name.c_str());

  print_timeStamp(kindsl_file);

  kindsl_file << "/*\n * Robot base\n */" << std::endl;
  kindsl_file << "Robot " << robot_name_ << " {" << std::endl;
  kindsl_file << "RobotBase " << root_link_name_;
  if (floating_base_) {
    kindsl_file << " floating";
  }
  kindsl_file << " {" << std::endl;
  kindsl_file << print_inertia_params(*urdf_model_.root_link_);
  kindsl_file << print_children(*urdf_model_.root_link_);
  kindsl_file << print_frame(*urdf_model_.root_link_);
  kindsl_file << "}" << std::endl;

  // links
  kindsl_file << "\n/*\n * Links\n */" << std::endl;
  unsigned int robcogen_link_id = 1;
  for (auto l_name : robot_link_names_) {
    if (l_name != root_link_name_) {
      kindsl_file << std::endl;
      kindsl_file << "link " << l_name << " {" << std::endl;
      kindsl_file << "\tid = " << robcogen_link_id++ << std::endl;
      if (debug_mode_) {
        std::cout << "printing inertia params for link " << l_name << std::endl;
        std::cout << print_inertia_params(*urdf_model_.links_[l_name]) << std::endl;
        std::cout << "printing frames for that same link" << std::endl;
        std::cout << print_frame(*urdf_model_.links_[l_name]) << std::endl;
      }
      kindsl_file << print_inertia_params(*urdf_model_.links_[l_name]);
      kindsl_file << print_children(*urdf_model_.links_[l_name]);
      kindsl_file << print_frame(*urdf_model_.links_[l_name]);
      kindsl_file << "}" << std::endl;
    }
  }

  // joints
  kindsl_file << "\n/*\n * Joints\n */" << std::endl;

  for (auto j_name : robot_joint_names_) {
    kindsl_file << std::endl;

    switch (urdf_model_.joints_[j_name]->type) {
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
    kindsl_file << "\t\t" << print_translation(*urdf_model_.joints_[j_name]);
    kindsl_file << "\t\t" << print_rotation(*urdf_model_.joints_[j_name]);
    kindsl_file << "\t}" << std::endl;
    kindsl_file << "}" << std::endl;
  }

  // end of robot
  kindsl_file << "\n}" << std::endl;

  kindsl_file.close();

  return true;
}

bool Urdf2RobCoGen::generateDtDsl() const {
  if (robot_link_names_.empty()) {
    throw std::runtime_error("Link names are still empty.");
  }

  std::string dtdsl_name = robot_name_ + ".dtdsl";
  std::ofstream dtdsl_file(dtdsl_name.c_str());

  print_timeStamp(dtdsl_file);

  dtdsl_file << "Robot " << robot_name_ << std::endl;
  dtdsl_file << "Frames {" << std::endl;
  dtdsl_file << list_frames() << std::endl;
  dtdsl_file << "}" << std::endl;

  dtdsl_file << "Transforms {" << std::endl;
  dtdsl_file << list_transforms() << std::endl;
  dtdsl_file << "}" << std::endl;

  dtdsl_file << "Jacobians {" << std::endl;
  dtdsl_file << list_jacobians() << std::endl;
  dtdsl_file << "}" << std::endl;

  dtdsl_file.close();

  return true;
}

std::vector<std::string> Urdf2RobCoGen::get_all_link_and_frame_names() const {
  std::vector<std::string> all_link_and_frame_names;
  all_link_and_frame_names.insert(all_link_and_frame_names.end(), robot_link_names_.begin(), robot_link_names_.end());
  std::for_each(frames_.begin(), frames_.end(),
                [&all_link_and_frame_names](std::pair<std::string, urdf::Joint> f) { all_link_and_frame_names.push_back(f.first); });
  return all_link_and_frame_names;
}

std::string Urdf2RobCoGen::list_frames() const {
  std::ostringstream out;

  std::vector<std::string> all_link_and_frame_names = get_all_link_and_frame_names();

  // All frames, 4 per line
  int k = 0;
  out << "\t";
  for (auto link_name : all_link_and_frame_names) {
    if (k > 3) {
      out << std::endl;
      out << "\t";
      k = 0;
    }
    out << "fr_" << link_name;
    if (link_name != all_link_and_frame_names.back()) {
      out << ", ";
    }
    k++;
  }
  return out.str();
}

std::string Urdf2RobCoGen::list_transforms() const {
  std::vector<std::string> all_link_and_frame_names = get_all_link_and_frame_names();

  std::ostringstream out;

  // request all transforms from and to the base
  for (const auto& to_name : all_link_and_frame_names) {
    if (root_link_name_ == to_name) {
      continue;
    }
    out << "\tbase=fr_" << root_link_name_ << ", target=fr_" << to_name << std::endl;
    out << "\tbase=fr_" << to_name << ", target=fr_" << root_link_name_ << std::endl;
  }

  return out.str();
}

std::string Urdf2RobCoGen::list_jacobians() const {
  // Note: Don't request jacobians from frames of the same link!
  // For now, generate Jacobians to all links and frames from the base

  std::ostringstream out;

  for (const auto& link : robot_link_names_) {
    if (link == root_link_name_) {
      continue;
    }
    out << "\tbase=fr_" << root_link_name_ << ", target=fr_" << link << std::endl;
  }

  for (const auto& frame : frames_) {
    if (frame.second.parent_link_name == root_link_name_) {
      continue;
    }
    out << "\tbase=fr_" << root_link_name_ << ", target=fr_" << frame.first << std::endl;
  }

  return out.str();
}

std::string Urdf2RobCoGen::print_inertia_params(const urdf::Link& link) const {
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
      << "\t\tCoM = "
      << "(0.0, 0.0, 0.0)" << std::endl
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

std::string Urdf2RobCoGen::print_children(const urdf::Link& link) {
  std::ostringstream out;
  bool style = false;
  out << "\tchildren {";
  for (std::string link_name : robot_link_names_) {
    if (link_name != root_link_name_) {
      const auto test_link = urdf_model_.links_[link_name];
      if (debug_mode_) {
        std::cout << "printing children of: " << link_name << std::endl;
        std::cout << "parent link of " << test_link->name << " is " << getParentLinkName(test_link) << std::endl;
      }

      if (getParentLinkName(test_link) == link.name) {
        if (!style) {
          out << std::endl;
          style = true;
        }
        out << "\t"
            << "\t" << test_link->name << " via " << test_link->parent_joint->name << std::endl;
      }
    }
  }
  if (style) out << "\t";
  out << "}" << std::endl;
  return out.str();
}

std::string Urdf2RobCoGen::print_translation(const urdf::Joint& joint) const {
  std::ostringstream out;
  // the position of the joint expressed in the parent_link frame
  out << "translation = " << print_vector(joint.parent_to_joint_origin_transform.position) << std::endl;
  return out.str();
}

std::string Urdf2RobCoGen::print_rotation(const urdf::Joint& joint) const {
  std::ostringstream out;
  // the orientation of the joint expressed in the parent_link frame
  out << "rotation = " << print_vector(rotation_vector(joint.parent_to_joint_origin_transform.rotation)) << std::endl;

  return out.str();
}

urdf::Vector3 Urdf2RobCoGen::rotation_vector(const urdf::Rotation& rot_quaternion) const {
  urdf::Vector3 rot_vec;
  Eigen::Matrix<double, 3, 1> p_min;
  kindr::RotationQuaternionD quat(rot_quaternion.w, rot_quaternion.x, rot_quaternion.y, rot_quaternion.z);

  kindr::EulerAnglesXyz<double> temp(quat);
  kindr::EulerAnglesXyz<double> xyz_unique = temp.getUnique();
  double x, y, z;
  double epsilon = 1e-4;
  std::fabs(xyz_unique.yaw()) < epsilon ? z = 0.0 : z = xyz_unique.yaw();
  std::fabs(xyz_unique.pitch()) < epsilon ? y = 0.0 : y = xyz_unique.pitch();
  std::fabs(xyz_unique.roll()) < epsilon ? x = 0.0 : x = xyz_unique.roll();

  rot_vec = urdf::Vector3(x, y, z);

  return rot_vec;
}

std::string Urdf2RobCoGen::print_frame(const urdf::Link& link) const {
  std::ostringstream out;
  bool print_title = false;
  for (std::pair<std::string, urdf::Joint> f : frames_) {
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

void Urdf2RobCoGen::print_timeStamp(std::ofstream& stream) const {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  stream << "/*\n * Autogenerated by Urdf2RobCoGen on " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "\n * DO NOT EDIT BY HAND\n */\n"
         << "" << std::endl;
}

std::string Urdf2RobCoGen::print_vector(const urdf::Vector3& vector) const {
  constexpr double epsilon = 1e-6;

  auto numberAsString = [](const double in) {
    if (std::abs(in - M_PI) < epsilon) {
      return std::string{"PI"};
    } else if (std::abs(in + M_PI) < epsilon) {
      return std::string{"-PI"};
    } else if (std::abs(in - M_PI_2) < epsilon) {
      return std::string{"PI/2.0"};
    } else if (std::abs(in + M_PI_2) < epsilon) {
      return std::string{"-PI/2.0"};
    } else if (std::abs(in - M_PI_4) < epsilon) {
      return std::string{"PI/4.0"};
    } else if (std::abs(in + M_PI_4) < epsilon) {
      return std::string{"-PI/4.0"};
    } else {
      std::ostringstream value_as_string;
      value_as_string.precision(std::numeric_limits<double>::digits10);
      value_as_string << std::fixed << in;
      return value_as_string.str();
    }
  };

  std::ostringstream out;
  out << "(" << numberAsString(vector.x) << "," << numberAsString(vector.y) << "," << numberAsString(vector.z) << ")";
  return out.str();
}

void Urdf2RobCoGen::printLinkJointNames() {
  std::cout << "Link Names: " << std::endl;
  for (auto l_name : robot_link_names_) {
    std::cout << l_name << std::endl;
  }

  std::cout << "\nJoint Names: " << std::endl;
  for (auto j_name : robot_joint_names_) {
    std::cout << j_name << std::endl;
  }

  std::cout << "Orphan Frames: " << std::endl;
  for (auto o_links : orphan_frames_) {
    std::cout << o_links << std::endl;
  }

  std::cout << "press <any key> to continue..." << std::endl;
  std::getchar();
}

std::string Urdf2RobCoGen::getParentLinkName(const urdf::LinkConstSharedPtr& link) {
  if (!link->getParent()->parent_joint || link->getParent()->parent_joint->type != urdf::Joint::FIXED) {
    return link->getParent()->name;
  } else {
    return getParentLinkName(link->getParent());
  }
}

}  // namespace urdf2robcogen
