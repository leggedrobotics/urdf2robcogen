#include <iostream>

#include <ros/ros.h>

#include <urdf2robcogen/Urdf2RobCoGen.hpp>

template<typename T>
T getParamWithCheck(const ros::NodeHandle& nodehandle, const std::string& name) {
  T value;
  if (!nodehandle.getParam(name, value)) {
    throw ros::Exception("Error reading ros parameter: " + name);
  }
  return value;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "urdf2robcogen_node");
  ros::NodeHandle nodeHandlePrivate("~");
  ros::NodeHandle nodeHandle;

  // Read parameters
  const auto robotName = getParamWithCheck<std::string>(nodeHandlePrivate, "robot_name");
  const auto outputFolder = getParamWithCheck<std::string>(nodeHandlePrivate, "output_folder");
  const auto descriptionName = getParamWithCheck<std::string>(nodeHandlePrivate, "description_name");
  const auto urdfString = getParamWithCheck<std::string>(nodeHandle, descriptionName);
  const auto verbose = getParamWithCheck<bool>(nodeHandlePrivate, "verbose");

  // Read urdf
  urdf::Model urdf;
  if (!urdf.initString(urdfString)) {
    throw std::runtime_error("Invalid URDF File");
  }
  TiXmlDocument urdfXml;  //! handle to the urdf xml file
  urdfXml.Parse(urdfString.c_str());

  // Run parser
  std::cout << "[urdf2robcogen] Start generating files." << std::endl;
  urdf2RobCoGen(robotName, std::move(urdf), std::move(urdfXml), outputFolder, verbose);
  std::cout << "[urdf2robcogen] Done generating files." << std::endl;

  return 0;
}
