/*! \file  Urdf2RobCoGen.hpp
 *  \brief Class for generating Kindsl files from URDF
 *  \authors jcarius, depardo
 *  \date Aug 29, 2017
 */

#pragma once

#include <tinyxml.h>
#include <urdf/model.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <memory>
#include <sstream>

namespace urdf2robcogen {

/*!
 * @brief This class allows users to generate the RobCoGen description files (.kindsl & .dtdsl)
 * from the Unifyed Robot Description Format (URDF) convention.
 */
class Urdf2RobCoGen {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  /*!
   * @brief Create a new URDF to Robcogen instance opening an urdf file
   * @param[in] path_to_urdf_file the path to the urdf file
   * @param[in] robot_name name of the robot
   * @param[in] debug_flag toggle verbose output and step by step execution
   */
  Urdf2RobCoGen(const std::string& path_to_urdf_file, const std::string& robot_name, bool debug_flag = false);

  //! Default destructor
  ~Urdf2RobCoGen() = default;

  /*!
   * @brief traverses the URDF kinematic tree structure to extract joint, link, and frame data
   */
  void parseUrdf();

  /*!
   * @brief Additional frames to be added to the links
   * @param frames a vector with the names of the frames (and end effectors)
   * @param parents as s vector with the names corresponding parent LINKS
   * @param translations a vector with the position of the frame w.r.t. parent frame
   * @param rotations a vector with the orientation of the frame w.r.t. parent frame
   */
  void setAdditionalFrames(const std::vector<std::string>& frames, const std::vector<std::string>& parents,
                           const std::vector<urdf::Vector3>& translations, const std::vector<urdf::Vector3>& rotations);

  /*!
   * @brief starts the generation process. Two files (.kindsl & .dtdsl are saved to disk at the end of this process)
   * @return a bool representing the success of the file generation
   */
  bool generateFiles();

 private:
  /*!
   * @brief recursively traverses the kinematic tree to fill internal joint, link, and frame objects
   * @param rootLink the urdf link corresponding to the root of the kinematic tree
   */
  void parseUrdfTreeRecursive(const urdf::LinkSharedPtr& rootLink);

  /*!
   * @brief transfers the inertia from a fixed urdf link to the closest movable parent
   * @param[in,out] link the link that has inertia but no movable parent joint
   */
  void moveInertiaFromFixedLink(const urdf::LinkSharedPtr& link);

  /*!
   * @brief helper method to extract 3x3 inertia matrix from a link
   * @param[in] link the urdf link from which the inertia is desired
   * @return a 3x3 inertia matrix of the link
   */
  Eigen::Matrix3d inertiaMatrixFromLink(const urdf::LinkConstSharedPtr& link);

  /*!
   * @brief helper method to assign inertia to a urdf link
   * @param[out] link the urdf link to which the new inertia will be assigned
   * @param[in] inertia the 3x3 inertia matrix to be assigned
   */
  void assignInertiaToLink(const urdf::LinkSharedPtr& link, const Eigen::Matrix3d& inertia);

  /*!
   * @brief helper method to construct a the skew symmetrix matrix (cross product matrix)
   * @param[in] vec the 3D vector
   * @return the corresponding skew symmetrix matrix (cross product matrix)
   */
  static Eigen::Matrix3d skewSymMatrixFromVector(const Eigen::Vector3d& vec);

  /*!
   * @brief Attach a fixed urdf link (=frame) to the closest movable parent link
   * @param[in] frameName name of the frame
   * @param[in] link the urdf link cooresponding to this frame
   * @param[in] q the quaternion representing any additional rotation to be applied to the frame
   * @param[in] r the position vector representing any additional offset to be applied to the frame
   */
  void attachFrameToLink(std::string frameName, const urdf::LinkConstSharedPtr& link, Eigen::Quaterniond q = Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d r = Eigen::Vector3d::Zero());

  /*!
   * @brief Recursively change joint frames to comply with kindsl conventions (rotation around z-axis)
   * for entire tree below the given link
   * @param[in,out] link the link from which the tree recursion should start to fix the joint orientation
   * @param[in] q_old_new the quaternion that was already applied to the parent link during recursion
   */
  void fixJointFrameRecursive(urdf::LinkSharedPtr& link, const Eigen::Quaterniond q_old_new = Eigen::Quaterniond::Identity());

  /*!
   * @brief change link position and rotation to be relative to true parent link
   * @param[in] jointName the name of the joint to fix
   */
  void fixJointParents(const std::string& jointName);

  /*!
   * @brief Name of parent link (true movable link)
   * @param[in] link query link whose parent we're looking for
   * @return the name of the movable parent link
   */
  std::string getParentLinkName(const urdf::LinkConstSharedPtr& link);



 private:
  //! converts a quaternion into roll pitch yaw representation
  urdf::Vector3 rotation_vector(const urdf::Rotation& rot_quaternion) const;

  //! print all child frames to a given link in kindsl format
  std::string print_frame(const urdf::Link& link) const;
  //! prints all child links to a given link in kindsl format
  std::string print_children(const urdf::Link& link);
  //! print translation in kindsl format
  std::string print_translation(const urdf::Joint& joint) const;
  //! print vector in kindsl format
  std::string print_vector(const urdf::Vector3& vector) const;
  //! print rotation in kindsl format
  std::string print_rotation(const urdf::Joint& joint) const;
  //! prints inertia parameters in kindsl format
  std::string print_inertia_params(const urdf::Link& link) const;
  //! print timestamp
  void print_timeStamp(std::ofstream& stream) const;

  //! print all frames in dtdsl format
  std::string list_frames() const;
  //! print all transforms in dtdsl format
  std::string list_transforms() const;
  //! print all jacobians in dtdsl format
  std::string list_jacobians() const;
  //! return all link and frame names
  std::vector<std::string> get_all_link_and_frame_names() const;

  //! Use the links in the URDF to set the relevant link and frame structure
  void setLinkNamesFromUrdf();

  /*!
   * @brief construct the kindsl file
   * @note this changes the internal model and cannot be called twice (!)
   */
  bool generateKindsl();

  //! construct the dtdsl file
  bool generateDtDsl() const;

  //! print link and joint names to console (for debugging)
  void printLinkJointNames();

 private:
  urdf::Model urdf_model_;  //! urdf parser object

  std::string robot_name_;                            //! name of the robot
  std::string root_link_name_;                        //! name of the robot's root link
  bool floating_base_;                                //! whether the robot is floating
  std::vector<std::string> robot_link_names_;         //! "true" links that will appear in the kindsl file
  std::map<std::string, unsigned int> urdf_link_id_;  //! stores the order in which links appear in the original urdf
  std::vector<std::string> robot_joint_names_;        //! "true" joints that will appear in the kindsl file
  std::map<std::string, urdf::Joint> frames_;         //! "fake" links that will appear as frames in the kindsl file
  std::vector<std::string> orphan_frames_;            //! temporary vector of frame names that will be added to frames_

  bool debug_mode_;  //! flag to turn on additional console output

  TiXmlDocument urdf_xml_;  //! handle to the urdf xml file
};

}  // namespace urdf2robcogen
