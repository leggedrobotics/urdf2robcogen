//
// Created by rgrandia on 23.09.19.
//

#include "urdf2robcogen/FixJointAxes.hpp"

#include "urdf2robcogen/utils.hpp"

void FixJointAxes(const urdf::Model& urdf, const UrdfInfo& urdfInfo, std::map<std::string, urdf::Joint>& fixedFrames) {
  auto jointPosesInRoot = getJointPosesInRoot(urdf, urdfInfo);
  const auto& jointKindslParent = urdfInfo.robotJointParentNames_;

  FixJointAxesRecursive(urdfInfo, urdf.root_link_, jointKindslParent, jointPosesInRoot, fixedFrames);
}

std::map<std::string, urdf::Pose> getJointPosesInRoot(const urdf::Model& urdf, const UrdfInfo& urdfInfo) {
  std::map<std::string, urdf::Pose> jointPosesInRoot;  //! joint positions and orientations in the root frame

  // Add root the poses in root
  urdf::Pose rootPose;
  rootPose.position = urdf::Vector3(0.0, 0.0, 0.0);
  rootPose.rotation = urdf::Rotation(0.0, 0.0, 0.0, 1.0);
  jointPosesInRoot.insert({urdf.root_link_->name, rootPose});

  generateJointPosesInRootRecursive(urdf.root_link_, rootPose, jointPosesInRoot);

  return jointPosesInRoot;
}

void generateJointPosesInRootRecursive(const urdf::LinkSharedPtr& link, const urdf::Pose& parentPoseInRoot,
                                       std::map<std::string, urdf::Pose>& jointPosesInRoot) {
  if (!link) {
    throw std::runtime_error("link pointer not valid");
  }

  for (const auto& child : link->child_links) {
    const auto& joint = child->parent_joint;
    auto childName = child->name;
    auto jointName = joint->name;

    // position
    auto p_joint_parent = joint->parent_to_joint_origin_transform.position;
    const urdf::Vector3 p_joint_root = p_joint_parent + parentPoseInRoot.position;

    // Orientation
    auto q_joint_parent = joint->parent_to_joint_origin_transform.rotation;
    const urdf::Rotation q_joint_root = q_joint_parent * parentPoseInRoot.rotation;

    // Store result
    urdf::Pose pose;
    pose.position = p_joint_root;
    pose.rotation = q_joint_root;
    jointPosesInRoot.insert({jointName, pose});

    ROS_DEBUG_STREAM("[generateJointPoses] Joint " << jointName << "\n");
    auto x_axis = q_joint_root * urdf::Vector3(1.0, 0.0, 0.0);
    auto z_axis = q_joint_root * urdf::Vector3(0.0, 0.0, 1.0);
    ROS_DEBUG_STREAM("\t old x-axis in root " << x_axis.x << ", " << x_axis.y << ", " << x_axis.z);
    ROS_DEBUG_STREAM("\t old z-axis in root " << z_axis.x << ", " << z_axis.y << ", " << z_axis.z);

    // Traverse down the tree
    generateJointPosesInRootRecursive(child, pose, jointPosesInRoot);
  }
}

urdf::Rotation getRelativeRotationInParentFrame(const urdf::Rotation& rotationParentInFrame, const urdf::Rotation& rotationChildInFrame) {
  //  return rotationChildInFrame * rotationParentInFrame.GetInverse();
  return rotationParentInFrame.GetInverse() * rotationChildInFrame;
}

void FixJointAxesRecursive(const UrdfInfo& urdfInfo, const urdf::LinkSharedPtr& link,
                           const std::map<std::string, std::string>& jointKindslParent, std::map<std::string, urdf::Pose>& jointPosesInRoot,
                           std::map<std::string, urdf::Joint>& fixedFrames) {
  if (!link) {
    throw std::runtime_error("link pointer not valid");
  }

  for (const auto& child : link->child_links) {
    const auto& joint = child->parent_joint;
    auto childName = child->name;
    auto jointName = joint->name;

    // Retreive
    const auto& kindslParent = jointKindslParent.at(jointName);
    const auto& parentPose_inRoot = jointPosesInRoot.at(kindslParent);
    auto& childPose_inRoot = jointPosesInRoot.at(jointName);

    // Orientation
    const auto rotation_parent_root_inRoot = parentPose_inRoot.rotation;
    const auto rotation_child_root_inRoot = childPose_inRoot.rotation;
    const auto rotation_child_parent_inRoot = getRelativeRotationInParentFrame(rotation_parent_root_inRoot, rotation_child_root_inRoot);

    // verify it is a true joint and not a frame
    if (std::find(urdfInfo.robotJointNames_.begin(), urdfInfo.robotJointNames_.end(), jointName) == urdfInfo.robotJointNames_.end()) {
      // This child is not a moving joint can directly set the orientation
      joint->parent_to_joint_origin_transform.rotation = rotation_child_parent_inRoot;
    } else {
      // This child is a moving joint --> will fix its orientation
      const auto oldChild_axis_root = rotation_child_root_inRoot * joint->axis;

      const Eigen::Vector3d oldChild_axis = toEigen(oldChild_axis_root);  // the original rotation axis in joint(=child) frame

      // define rotation how old child frame must be rotated such that axis aligns with z
      const Eigen::Quaterniond q_newChild_root_eigen = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), oldChild_axis);
      const urdf::Rotation q_newChild_root_inRoot = fromEigen(q_newChild_root_eigen);

      // Fix orientations
      const auto new_rotation_child_root_inRoot = q_newChild_root_inRoot;
      const auto new_rotation_child_parent_inParent =
          getRelativeRotationInParentFrame(rotation_parent_root_inRoot, new_rotation_child_root_inRoot);
      childPose_inRoot.rotation = new_rotation_child_root_inRoot;
      ROS_DEBUG_STREAM("[FixJointAxesRecursive] Joint " << jointName);
      ROS_DEBUG_STREAM("\t old x-axis in root " << printVector(rotation_child_root_inRoot * urdf::Vector3(1.0, 0.0, 0.0)));
      ROS_DEBUG_STREAM("\t new x-axis in root " << printVector(new_rotation_child_root_inRoot * urdf::Vector3(1.0, 0.0, 0.0)));
      ROS_DEBUG_STREAM("\t old z-axis in root " << printVector(rotation_child_root_inRoot * urdf::Vector3(0.0, 0.0, 1.0)));
      ROS_DEBUG_STREAM("\t new z-axis in root " << printVector(new_rotation_child_root_inRoot * urdf::Vector3(0.0, 0.0, 1.0)));
      ROS_DEBUG_STREAM("\t rotation root to child" << printVector(rotationXyz(new_rotation_child_root_inRoot)));
      ROS_DEBUG_STREAM("\t rotation root to parent" << printVector(rotationXyz(rotation_parent_root_inRoot)));
      ROS_DEBUG_STREAM("\t rotation parent to child" << printVector(rotationXyz(new_rotation_child_parent_inParent)));

      // set fixed orientation
      joint->parent_to_joint_origin_transform.rotation = new_rotation_child_parent_inParent;

      // Correct inertia of next link after rotating joint
      if (child->inertial) {
        ROS_DEBUG_STREAM("[FixJointAxesRecursive] Correcting inertia of " << childName);

        // Extract local rotation performed
        // This child is a moving joint --> will fix its orientation
        Eigen::Vector3d oldChild_axis_inParent = toEigen(joint->axis);  // the original rotation axis in joint(=child) frame

        const urdf::Rotation q_newChild_oldChild =
            getRelativeRotationInParentFrame(rotation_child_root_inRoot, new_rotation_child_root_inRoot);
        const Eigen::Quaterniond q_newChild_oldChild_eigen = toEigen(q_newChild_oldChild);
        Eigen::Matrix3d R_old_new = q_newChild_oldChild_eigen.toRotationMatrix();  // from active to passive rotation?
        Eigen::Matrix3d R_new_old = R_old_new.transpose();                         // from active to passive rotation?
        ROS_DEBUG_STREAM("\t rotation old Child to new Child" << printVector(rotationXyz(q_newChild_oldChild)));
        ROS_DEBUG_STREAM("\t matrix old Child to new Child\n" << R_new_old);
        ROS_DEBUG_STREAM("\t old rotation axis mapped to new Child " << (R_new_old * oldChild_axis_inParent).transpose()
                                                                     << " should be == [0, 0, 1]");

        // Rotate center of mass position
        const Eigen::Vector3d com_inOldChild =
            toEigen(child->inertial->origin.position);  // the original rotation axis in joint(=child) frame
        const Eigen::Vector3d com_inNewChild = R_new_old * com_inOldChild;
        child->inertial->origin.position = fromEigen(com_inNewChild);

        // Need to also update the COM frame
        auto& comFrame = fixedFrames[childName + "_COM"];
        comFrame.parent_to_joint_origin_transform.position = child->inertial->origin.position;

        // Rotate inertia
        // rotate child inertia into parent frame
        Eigen::Matrix3d oldChild_I = inertiaMatrixFromLink(child);
        const Eigen::Matrix3d newChild_I = R_new_old * oldChild_I * R_new_old.transpose();
        assignInertiaToLink(child, newChild_I);

        ROS_DEBUG_STREAM("\t old com " << com_inOldChild.transpose());
        ROS_DEBUG_STREAM("\t new com " << com_inNewChild.transpose());
        ROS_DEBUG_STREAM("\t old Child Inertia \n" << oldChild_I);
        ROS_DEBUG_STREAM("\t new Child Inertia \n" << newChild_I);
      }
    }

    // Position
    urdf::Vector3 position_child_parent_inRoot(childPose_inRoot.position.x - parentPose_inRoot.position.x,
                                               childPose_inRoot.position.y - parentPose_inRoot.position.y,
                                               childPose_inRoot.position.z - parentPose_inRoot.position.z);
    const auto position_child_parent_inParent = rotation_parent_root_inRoot.GetInverse() * position_child_parent_inRoot;
    joint->parent_to_joint_origin_transform.position = position_child_parent_inParent;

    // Update frame with the same
    for (auto& frame : fixedFrames) {
      auto& frameName = frame.first;
      auto& frameJoint = frame.second;
      if (frameName == childName) {
        frameJoint.parent_to_joint_origin_transform.position = joint->parent_to_joint_origin_transform.position;
        frameJoint.parent_to_joint_origin_transform.rotation = joint->parent_to_joint_origin_transform.rotation;
      }
    }

    // Traverse down the tree
    FixJointAxesRecursive(urdfInfo, child, jointKindslParent, jointPosesInRoot, fixedFrames);
  }
}