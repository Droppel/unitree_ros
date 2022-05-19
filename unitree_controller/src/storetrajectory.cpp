#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/StateLin3d.h>
#include <geometry_msgs/Vector3.h>
#include "ros/ros.h"

void motionToNode(YAML::Emitter& em, xpp_msgs::RobotStateCartesianTrajectory motion, std::string name)
{
	em << YAML::BeginMap;

	//write header
	em << YAML::Key << "header";
	em << YAML::Value << YAML::BeginMap;

	em << YAML::Key << "name" << YAML::Value << name;
	em << YAML::EndMap;
	
    auto frames = motion.points;
	//write motion
	em << YAML::Key << "motion" << YAML::Value << YAML::BeginSeq;
	for (int i = 0; i < (int)frames.size(); i++)
	{
		em << YAML::BeginMap; //Element
        em << YAML::Key << "time_from_start" << YAML::Value << frames[i].time_from_start.toSec();
		
        em << YAML::Key << "base" << YAML::Value << YAML::BeginMap; // Base
        em << YAML::Key << "pose" << YAML::Value << YAML::BeginMap; // Pose
        em << YAML::Key << "position" << YAML::Value << YAML::BeginMap; // Position
        em << YAML::Key << "x" << YAML::Value << frames[i].base.pose.position.x;
        em << YAML::Key << "y" << YAML::Value << frames[i].base.pose.position.y;
        em << YAML::Key << "z" << YAML::Value << frames[i].base.pose.position.z;
        em << YAML::EndMap; // End Position
        em << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap; // Orientation
        em << YAML::Key << "x" << YAML::Value << frames[i].base.pose.orientation.x;
        em << YAML::Key << "y" << YAML::Value << frames[i].base.pose.orientation.y;
        em << YAML::Key << "z" << YAML::Value << frames[i].base.pose.orientation.z;
        em << YAML::Key << "w" << YAML::Value << frames[i].base.pose.orientation.w;
        em << YAML::EndMap; // End Orientation
        em << YAML::EndMap; // End Pose
        em << YAML::Key << "twist" << YAML::Value << YAML::BeginMap; // Twist
        em << YAML::Key << "linear" << YAML::Value << YAML::BeginMap; // Linear
        em << YAML::Key << "x" << YAML::Value << frames[i].base.twist.linear.x;
        em << YAML::Key << "y" << YAML::Value << frames[i].base.twist.linear.y;
        em << YAML::Key << "z" << YAML::Value << frames[i].base.twist.linear.z;
        em << YAML::EndMap; // End Linear
        em << YAML::Key << "angular" << YAML::Value << YAML::BeginMap; // angular
        em << YAML::Key << "x" << YAML::Value << frames[i].base.twist.angular.x;
        em << YAML::Key << "y" << YAML::Value << frames[i].base.twist.angular.y;
        em << YAML::Key << "z" << YAML::Value << frames[i].base.twist.angular.z;
        em << YAML::EndMap; // End angular
        em << YAML::EndMap; // End Twist
        em << YAML::Key << "accel" << YAML::Value << YAML::BeginMap; // accel
        em << YAML::Key << "linear" << YAML::Value << YAML::BeginMap; // Linear
        em << YAML::Key << "x" << YAML::Value << frames[i].base.accel.linear.x;
        em << YAML::Key << "y" << YAML::Value << frames[i].base.accel.linear.y;
        em << YAML::Key << "z" << YAML::Value << frames[i].base.accel.linear.z;
        em << YAML::EndMap; // End Linear
        em << YAML::Key << "angular" << YAML::Value << YAML::BeginMap; // angular
        em << YAML::Key << "x" << YAML::Value << frames[i].base.accel.angular.x;
        em << YAML::Key << "y" << YAML::Value << frames[i].base.accel.angular.y;
        em << YAML::Key << "z" << YAML::Value << frames[i].base.accel.angular.z;
        em << YAML::EndMap; // End angular        
        em << YAML::EndMap; // End accel 
        em << YAML::EndMap; // End Base


        em << YAML::Key << "ee_motion" << YAML::Value << YAML::BeginSeq; //Begin EE_Motion
        for (int k = 0; k < (int)frames[i].ee_motion.size(); k++)
        {
            em << YAML::BeginMap; // element
            em << YAML::Key << "pos" << YAML::Value << YAML::BeginMap; // pos
            em << YAML::Key << "x" << YAML::Value << frames[i].ee_motion[k].pos.x;
            em << YAML::Key << "y" << YAML::Value << frames[i].ee_motion[k].pos.y;
            em << YAML::Key << "z" << YAML::Value << frames[i].ee_motion[k].pos.z;
            em << YAML::EndMap; // End pos
            em << YAML::Key << "vel" << YAML::Value << YAML::BeginMap; // vel
            em << YAML::Key << "x" << YAML::Value << frames[i].ee_motion[k].vel.x;
            em << YAML::Key << "y" << YAML::Value << frames[i].ee_motion[k].vel.y;
            em << YAML::Key << "z" << YAML::Value << frames[i].ee_motion[k].vel.z;
            em << YAML::EndMap; // End vel
            em << YAML::Key << "acc" << YAML::Value << YAML::BeginMap; // acc
            em << YAML::Key << "x" << YAML::Value << frames[i].ee_motion[k].acc.x;
            em << YAML::Key << "y" << YAML::Value << frames[i].ee_motion[k].acc.y;
            em << YAML::Key << "z" << YAML::Value << frames[i].ee_motion[k].acc.z;
            em << YAML::EndMap; // End acc
            em << YAML::EndMap; // End element
        }
        em << YAML::EndSeq; // End EE_Motion

        em << YAML::Key << "ee_forces" << YAML::Value << YAML::BeginSeq; //Begin EE_Forces
        for (int k = 0; k < (int)frames[i].ee_forces.size(); k++)
        {
            em << YAML::BeginMap; // element
            em << YAML::Key << "x" << YAML::Value << frames[i].ee_forces[k].x;
            em << YAML::Key << "y" << YAML::Value << frames[i].ee_forces[k].y;
            em << YAML::Key << "z" << YAML::Value << frames[i].ee_forces[k].z;
            em << YAML::EndMap; // End element
        }
        em << YAML::EndSeq; // End EE_Forces

        em << YAML::Key << "ee_contact" << YAML::Value << YAML::BeginSeq; //Begin EE_Contact
        for (int k = 0; k < (int)frames[i].ee_forces.size(); k++)
        {
            em << YAML::Value << std::to_string(frames[i].ee_contact[k]);
        }
        em << YAML::EndSeq; // End EE_Contact

		em << YAML::EndMap; // End Element
	}
	em << YAML::EndSeq;

	em << YAML::EndMap;
}

bool save(std::string name, xpp_msgs::RobotStateCartesianTrajectory motion)
{

	YAML::Emitter em;

	motionToNode(em, motion, name);

	std::ofstream out;

	out.open(name.c_str());
	out << em.c_str() << "\n";
	out.close();
	return true;
}

bool nodeToMotion(YAML::Node node, xpp_msgs::RobotStateCartesianTrajectory* msg) {
    
    YAML::Node motion = node["motion"];

	//write motion
    msg->points.reserve((int)motion.size());
	for (int i = 0; i < (int)motion.size(); i++)
	{
		xpp_msgs::RobotStateCartesian point = xpp_msgs::RobotStateCartesian();
        point.time_from_start = ros::Duration(motion[i]["time_from_start"].as<double>());

        YAML::Node basepose = motion[i]["base"]["pose"];

        point.base.pose.position.x = basepose["position"]["x"].as<double>();
        point.base.pose.position.y = basepose["position"]["y"].as<double>();
        point.base.pose.position.z = basepose["position"]["z"].as<double>();
        point.base.pose.orientation.x = basepose["orientation"]["x"].as<double>();
        point.base.pose.orientation.y = basepose["orientation"]["y"].as<double>();
        point.base.pose.orientation.z = basepose["orientation"]["z"].as<double>();
        point.base.pose.orientation.w = basepose["orientation"]["w"].as<double>();

        YAML::Node basetwist = motion[i]["base"]["twist"];
        point.base.twist.linear.x = basetwist["linear"]["x"].as<double>();
        point.base.twist.linear.y = basetwist["linear"]["y"].as<double>();
        point.base.twist.linear.z = basetwist["linear"]["z"].as<double>();
        point.base.twist.angular.x = basetwist["angular"]["x"].as<double>();
        point.base.twist.angular.y = basetwist["angular"]["y"].as<double>();
        point.base.twist.angular.z = basetwist["angular"]["z"].as<double>();

        YAML::Node baseaccel = motion[i]["base"]["accel"];
        point.base.accel.linear.x = baseaccel["linear"]["x"].as<double>();
        point.base.accel.linear.y = baseaccel["linear"]["y"].as<double>();
        point.base.accel.linear.z = baseaccel["linear"]["z"].as<double>();
        point.base.accel.angular.x = baseaccel["angular"]["x"].as<double>();
        point.base.accel.angular.y = baseaccel["angular"]["y"].as<double>();
        point.base.accel.angular.z = baseaccel["angular"]["z"].as<double>();

        point.ee_motion.reserve((int)motion[i]["ee_motion"].size());
        for (int k = 0; k < (int)motion[i]["ee_motion"].size(); k++)
        {
            xpp_msgs::StateLin3d ee_motion = xpp_msgs::StateLin3d();
            YAML::Node ee_motionnode = motion[i]["ee_motion"][k];
            ee_motion.pos.x = ee_motionnode["pos"]["x"].as<double>();
            ee_motion.pos.y = ee_motionnode["pos"]["y"].as<double>();
            ee_motion.pos.z = ee_motionnode["pos"]["z"].as<double>();
            ee_motion.vel.x = ee_motionnode["vel"]["x"].as<double>();
            ee_motion.vel.y = ee_motionnode["vel"]["y"].as<double>();
            ee_motion.vel.z = ee_motionnode["vel"]["z"].as<double>();
            ee_motion.acc.x = ee_motionnode["acc"]["x"].as<double>();
            ee_motion.acc.y = ee_motionnode["acc"]["y"].as<double>();
            ee_motion.acc.z = ee_motionnode["acc"]["z"].as<double>();
            point.ee_motion.push_back(ee_motion);
        }

        point.ee_forces.reserve((int)motion[i]["ee_forces"].size());
        for (int k = 0; k < (int)motion[i]["ee_forces"].size(); k++)
        {
            geometry_msgs::Vector3 ee_force = geometry_msgs::Vector3();
            YAML::Node ee_forcenode = motion[i]["ee_forces"][k];
            ee_force.x = ee_forcenode["x"].as<double>();
            ee_force.y = ee_forcenode["y"].as<double>();
            ee_force.z = ee_forcenode["z"].as<double>();
            point.ee_forces.push_back(ee_force);
        }

        point.ee_contact.reserve((int)motion[i]["ee_contact"].size());
        for (int k = 0; k < (int)motion[i]["ee_contact"].size(); k++)
        {
            int val = motion[i]["ee_contact"][k].as<int>();
            point.ee_contact.push_back(val);
        }

        msg->points.push_back(point);
	}
    return true;
}

bool load(std::string name, xpp_msgs::RobotStateCartesianTrajectory* msg)
{
    ROS_INFO_STREAM(std::fixed << "Name " << name);
	YAML::Node node;
	try
	{
		node = YAML::LoadFile(name);
		return nodeToMotion(node, msg);
	}
	catch (YAML::Exception& e)
	{
		ROS_ERROR("Error: '%s' ", e.what());
		return false;
	}
	return true;
}