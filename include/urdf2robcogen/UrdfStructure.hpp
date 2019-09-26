//
// Created by rgrandia on 24.09.19.
//

#pragma once

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

struct LinkInfo;
struct JointInfo;
struct FrameInfo;

struct UrdfStructure {
  // Robot properties
  bool floatingBaseSystem_;
  std::string robotName_;
  std::string rootLinkName_;

  // Components
  std::map<std::string, LinkInfo> links_; //! Pair of link name and corresponding information
  std::map<std::string, JointInfo> joints_; //! Pair of joint name and corresponding information
  std::map<std::string, FrameInfo> frames_; //! Pair of frame name and corresponding information
};


struct PoseInWorld {
  Eigen::Vector3d position_; //! position from world origin to frame origin.
  Eigen::Quaterniond rotationWorldToFrame_; //! rotates a vector from world to frame.
};

struct Inertia {
  PoseInWorld comFramePoseInWorld_; //! Pose of the COM frame w.r.t. the World frame.
  double m_; //! Mass
  Eigen::Matrix3d I_; //! Inertia tensor in the local com frame defined by comFramePoseInWorld_
};

struct LinkInfo {
  int id; //! id contains the order in which the links were added to the urdfstructure
  std::string parentJoint_;
  std::vector<std::string> childLinks_;
  std::unique_ptr<Inertia> inertia_;
  // Does not store a pose. the link frame is equal to that of the parentJoint

  //! Constructor
  LinkInfo() : id(-1), parentJoint_(), childLinks_(), inertia_(new Inertia()) {}

  //! Copy constructor
  LinkInfo(const LinkInfo& obj) : id(obj.id), parentJoint_(obj.parentJoint_), childLinks_(obj.childLinks_), inertia_(new Inertia(*obj.inertia_)) {}

  //! Copy assignment
  LinkInfo& operator=(const LinkInfo& obj) {
    id = obj.id;
    parentJoint_ = obj.parentJoint_;
    childLinks_ = obj.childLinks_;
    inertia_.reset(new Inertia(*obj.inertia_));
    return *this;
  }

  //! Move constructor
  LinkInfo(LinkInfo&& obj) : id(obj.id), parentJoint_(std::move(obj.parentJoint_)), childLinks_(std::move(obj.childLinks_)), inertia_(std::move(obj.inertia_)) {}

  //! Move assignment
  LinkInfo& operator=(LinkInfo&& obj) {
    id = obj.id;
    parentJoint_ = std::move(obj.parentJoint_);
    childLinks_ = std::move(obj.childLinks_);
    inertia_ = std::move(obj.inertia_);
    return *this;
  }
};

enum class JointType { fixed, r_joint, p_joint};

struct JointInfo {
  int id; //! id contains the order in which the joints were added to the urdfstructure
  std::string parentLink_;
  JointType type_;
  PoseInWorld poseInWorld_;
  Eigen::Vector3d axisInJointFrame_;
};

struct FrameInfo {
  std::string parentLinkName_;
  PoseInWorld poseInWorld_;
};

std::string toString(JointType jointType);

void printLinkJointStructure(const UrdfStructure& urdfStructure);
