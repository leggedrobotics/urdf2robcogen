//
// Created by rgrandia on 23.09.19.
//

#pragma once

#include <urdf/model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Matrix3d inertiaMatrixFromLink(const urdf::LinkConstSharedPtr& link);

void assignInertiaToLink(const urdf::LinkSharedPtr& link, const Eigen::Matrix3d& inertia);

Eigen::Matrix3d skewSymMatrixFromVector(const Eigen::Vector3d& vec);

Eigen::Vector3d toEigen(const urdf::Vector3& vec);
urdf::Vector3 fromEigen(const Eigen::Vector3d& vec);

Eigen::Quaterniond toEigen(const urdf::Rotation& rotation);
urdf::Rotation fromEigen(const Eigen::Quaterniond& rotation);

std::string printVector(const urdf::Vector3& vector);
urdf::Vector3 rotationXyz(const urdf::Rotation& rot_quaternion);

void print_timeStamp(std::ofstream& stream);