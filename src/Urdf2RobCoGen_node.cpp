#include <urdf2robcogen/Urdf2RobCoGen.hpp>

int main(int argc, char* argv[]) {
  std::cout << "This is urdf2robocogen." << std::endl;

  if (argc < 3) {
    std::cout << "Usage:\n"
              << "./urdf2robcogen_node RobotName /path/to/description.urdf" << std::endl;
    std::cout << "Going to exit." << std::endl;
    return 0;
  }

  const std::string robotName = argv[1];
  const std::string urdfPath = argv[2];

  constexpr bool debug_on = false;

  urdf2robcogen::Urdf2RobCoGen myrobot(urdfPath, robotName, debug_on);
  myrobot.parseUrdf();
  myrobot.generateFiles();

  std::cout << "urdf2robcogen: Done generating files." << std::endl;

  return 0;
}
