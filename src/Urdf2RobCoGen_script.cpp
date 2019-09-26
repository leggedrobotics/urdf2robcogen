#include <urdf2robcogen/Urdf2RobCoGen.hpp>

#include <iostream>

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cout << "Usage:\n"
              << "./urdf2robcogen_node RobotName /path/to/description.urdf (optional: -v)" << std::endl;
    std::cout << "Going to exit." << std::endl;
    return 0;
  }

  // Get Parameters
  const std::string robotName = argv[1];
  const std::string urdfPath = argv[2];
  const std::string outputFolder = "";
  const bool verbose = (argc >= 4) && (std::string(argv[3]) == "-v");

  // Read URDF
  urdf::Model urdf;
  if (!urdf.initFile(urdfPath)) {
    throw std::runtime_error("Invalid URDF File");
  }
  TiXmlDocument urdfXml;
  urdfXml.LoadFile(urdfPath);

  // Run parser
  std::cout << "[urdf2robcogen] Start generating files." << std::endl;
  urdf2RobCoGen(robotName, std::move(urdf), std::move(urdfXml), outputFolder, verbose);
  std::cout << "[urdf2robcogen] Done generating files." << std::endl;

  return 0;
}
