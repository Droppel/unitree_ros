#include <string>
#include <yaml-cpp/yaml.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>

bool save(std::string name, xpp_msgs::RobotStateCartesianTrajectory motion);
void motionToNode(YAML::Emitter& em, xpp_msgs::RobotStateCartesianTrajectory motion, std::string name);
bool nodeToMotion(YAML::Node node, xpp_msgs::RobotStateCartesianTrajectory* msg);
bool load(std::string name, xpp_msgs::RobotStateCartesianTrajectory* msg);