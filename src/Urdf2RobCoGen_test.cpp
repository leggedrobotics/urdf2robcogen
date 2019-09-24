#include <urdf2robcogen/Urdf2RobCoGen.hpp>

#include <urdf2robcogen/parseUrdf.hpp>

int main(int argc, char* argv[]) {
  std::cout << "This is urdf2robocogen." << std::endl;

  const std::string robotName = "anymal_c";
  const std::string urdfPath = "/home/rgrandia/projects/anymal_c/catkin_ws/src/urdf2robcogen/anymal_c.urdf";

  constexpr bool debug_on = true;

  urdf2robcogen::Urdf2RobCoGen myrobot(urdfPath, robotName, debug_on);
  myrobot.parseUrdf();
  myrobot.generateFiles();

  std::cout << "urdf2robcogen: Done generating files." << std::endl;

  parseUrdf(robotName + "_v2", urdfPath);

  return 0;
}
