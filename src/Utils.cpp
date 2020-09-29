//
// Created by rgrandia on 23.09.19.
//

#include "urdf2robcogen/Utils.hpp"

#include <kindr/Core>

#include <ctime>
#include <fstream>

Eigen::Matrix3d inertiaMatrixFromLink(const urdf::Link& link) {
  Eigen::Matrix3d inertia;

  inertia(0, 0) = link.inertial->ixx;
  inertia(1, 1) = link.inertial->iyy;
  inertia(2, 2) = link.inertial->izz;
  inertia(0, 1) = inertia(1, 0) = link.inertial->ixy;
  inertia(0, 2) = inertia(2, 0) = link.inertial->ixz;
  inertia(1, 2) = inertia(2, 1) = link.inertial->iyz;

  assert(isValidInertiaMatrix(inertia));

  return inertia;
}

bool isValidInertiaMatrix(Eigen::Matrix3d inertia) {
  bool isValid = true;

  Eigen::VectorXcd eigenvalues = inertia.eigenvalues();
  isValid &= (eigenvalues.real().minCoeff() > -1e-9);
  isValid &= (eigenvalues.imag().cwiseAbs().maxCoeff() < 1e-9);

  isValid &= (std::fabs(inertia(0, 1) - inertia(1, 0)) < 1e-6);
  isValid &= (std::fabs(inertia(0, 2) - inertia(2, 0)) < 1e-6);
  isValid &= (std::fabs(inertia(1, 2) - inertia(2, 1)) < 1e-6);

  return isValid;
}

Eigen::Matrix3d expressInertiaFromFrameAInComFrame(Eigen::Matrix3d A_I_A, double m, PoseInWorld poseA, PoseInWorld poseCom) {
  // Notation A_I_C = inertia w.r.t point C expressed in frame A
  // Relative orientation
  const Eigen::Quaterniond& q_A_W = poseA.rotationWorldToFrame_;
  const Eigen::Quaterniond& q_C_W = poseCom.rotationWorldToFrame_;
  const Eigen::Quaterniond& q_A_C = q_A_W * q_C_W.inverse();  // rotates a vector from frame C to A
  const Eigen::Matrix3d R_A_C = q_A_C.toRotationMatrix();     // rotates a vector from frame C to A

  // Relative position
  const Eigen::Vector3d& W_r_A_C = poseA.position_ - poseCom.position_;  // vector from C to A in world frame
  const Eigen::Vector3d C_r_A_C = q_C_W * W_r_A_C;                       // vector from C to A in C frame

  // Rotate inertia into frame C
  const Eigen::Matrix3d C_I_A = R_A_C.transpose() * A_I_A * R_A_C;

  // Translation term from frame A to C
  const Eigen::Matrix3d C_steinerTerm_C = m * skewSymMatrixFromVector(C_r_A_C) * skewSymMatrixFromVector(C_r_A_C).transpose();

  // Combined inertia
  Eigen::Matrix3d C_I_C = C_I_A + C_steinerTerm_C;

  // Check if still a valid inertia tensor
  assert(isValidInertiaMatrix(C_I_C));
  return C_I_C;
}

Eigen::Matrix3d skewSymMatrixFromVector(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d skew;
  skew << 0, -vec(2), vec(1),  // clang-format off
          vec(2), 0, -vec(0),
          -vec(1), vec(0), 0;  // clang-format on
  return skew;
}

Eigen::Vector3d toEigen(const urdf::Vector3& vec) {
  return {vec.x, vec.y, vec.z};
}

urdf::Vector3 fromEigen(const Eigen::Vector3d& vec) {
  return {vec(0), vec(1), vec(2)};
}

Eigen::Quaterniond toEigen(const urdf::Rotation& rotation) {
  Eigen::Quaterniond q;
  rotation.getQuaternion(q.x(), q.y(), q.z(), q.w());
  return q;
}

urdf::Rotation fromEigen(const Eigen::Quaterniond& rotation) {
  return {rotation.x(), rotation.y(), rotation.z(), rotation.w()};
}

std::string printVector(const urdf::Vector3& vector) {
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

urdf::Vector3 rotationXyz(const urdf::Rotation& rot_quaternion) {
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

std::pair<Eigen::Vector3d, Eigen::Quaterniond> getRelativePose(const PoseInWorld& poseChild, const PoseInWorld& poseParent) {
  // Orientation
  const auto& q_C_W = poseChild.rotationWorldToFrame_;
  const auto& q_P_W = poseParent.rotationWorldToFrame_;
  const Eigen::Quaterniond q_P_C = q_P_W * q_C_W.inverse();

  // Position
  const auto& W_p_C_W = poseChild.position_;
  const auto& W_p_P_W = poseParent.position_;
  const Eigen::Vector3d P_p_C_P = q_P_W.toRotationMatrix() * (W_p_C_W - W_p_P_W);

  return {P_p_C_P, q_P_C};
}

std::string print_timeStamp() {
  std::ostringstream out;

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  out << "/*\n * Autogenerated by Urdf2RobCoGen on " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "\n * DO NOT EDIT BY HAND\n */\n"
      << "" << std::endl;

  return out.str();
}
