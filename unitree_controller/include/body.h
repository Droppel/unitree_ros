/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#include <sensor_msgs/JointState.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <Eigen/Core>
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern unitree_legged_msgs::LowCmd lowCmd;
extern unitree_legged_msgs::LowState lowState;

void stand();
void motion_init();
void moveTowr(sensor_msgs::JointState sensormsg, xpp_msgs::RobotStateCartesian trajectory, double duration, int counter);
void sendServoCmd(double sleepAfter);
void moveAllPosition(double* jointPositions, double duration);
void newMoveAllPosition(xpp_msgs::RobotStateCartesian trajectory, double* jointPositions, double duration, int counter);
void getJacobian(int limb, Eigen::MatrixXd& J, double* jointPositions);
void limbFTtoJointTorque(int limb, double* jointPositions, xpp_msgs::RobotStateCartesian trajectory);
void setLimbTorques(int limb, Eigen::MatrixXd T, double* jointPositions);
double coerce(double x, double min, double max);
}

#endif
