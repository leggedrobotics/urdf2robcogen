//
// Created by rgrandia on 23.09.19.
//

#pragma once

#include <urdf/model.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <urdf2robcogen/UrdfStructure.hpp>

Eigen::Matrix3d inertiaMatrixFromLink(const urdf::Link& link);
bool isValidInertiaMatrix(Eigen::Matrix3d I);
Eigen::Matrix3d expressInertiaFromFrameAInComFrame(Eigen::Matrix3d A_I_A, double m, PoseInWorld poseA, PoseInWorld poseC);

Eigen::Matrix3d skewSymMatrixFromVector(const Eigen::Vector3d& vec);

Eigen::Vector3d toEigen(const urdf::Vector3& vec);
urdf::Vector3 fromEigen(const Eigen::Vector3d& vec);

Eigen::Quaterniond toEigen(const urdf::Rotation& rotation);
urdf::Rotation fromEigen(const Eigen::Quaterniond& rotation);

std::string printVector(const urdf::Vector3& vector);

urdf::Vector3 rotationXyz(const urdf::Rotation& rot_quaternion);

std::pair<Eigen::Vector3d, Eigen::Quaterniond> getRelativePose(const PoseInWorld& poseChild, const PoseInWorld& poseParent);

std::string print_timeStamp();

template <typename K, typename T>
std::vector<K> getKeys(std::map<K, T> m) {
  std::vector<K> v;
  v.reserve(m.size());
  for (const auto& keyValue : m) {
    v.push_back(keyValue.first);
  }
  return v;
}

template <typename T>
std::vector<std::pair<std::string, T>> toIDSortedVector(std::map<std::string, T> m) {
  std::vector<std::pair<std::string, T>> v;
  v.reserve(m.size());
  for (const auto& keyValue : m) {
    v.push_back(keyValue);
  }

  std::sort(v.begin(), v.end(),
            [](const std::pair<std::string, T>& a, const std::pair<std::string, T>& b) { return a.second.id < b.second.id; });
  return v;
}

template <typename K>
void removeFromContainer(const K& key, std::vector<K>& container) {
  auto keyIterator = std::find(container.begin(), container.end(), key);
  if (keyIterator != container.end()) {
    container.erase(keyIterator);
  } else {
    throw std::runtime_error("Key to be deleted not found");
  }
}

template <typename K, typename T>
void removeFromContainer(const K& key, std::map<K, T>& container) {
  auto keyIterator = container.find(key);
  if (keyIterator != container.end()) {
    container.erase(keyIterator);
  } else {
    throw std::runtime_error("Key to be deleted not found");
  }
}