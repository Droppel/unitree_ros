/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <xpp_vis/inverse_kinematics.h>
#include <Eigen/Dense>
#include <xpp_msgs/RobotStateCartesian.h>

enum HyqJointID {HAA=0, HFE, KFE, HyqlegJointCount};

/**
 * @brief Returns joint angles to reach for a specific foot position.
 * @param pos_B  3D-position of the foot expressed in the base frame (B).
 */
xpp::Joints GetAllJointAngles(const xpp_msgs::RobotStateCartesian& x_B);

/**
 * @brief Number of endeffectors (feet, hands) this implementation expects.
 */
int GetEECount() { return 4; };

Eigen::Vector3d base2hip_LF_ = Eigen::Vector3d(0.2399, 0.051, 0.0);

enum KneeBend { Forward, Backward };

/**
 * @brief Returns the joint angles to reach a Cartesian foot position.
 * @param ee_pos_H  Foot position xyz expressed in the frame attached
 * at the hip-aa (H).
 */
Eigen::Vector3d GetJointAngles(const Eigen::Vector3d& ee_pos_H, bool leftside, bool front);

/**
 * @brief Restricts the joint angles to lie inside the feasible range
 * @param q[in/out]  Current joint angle that is adapted if it exceeds
 * the specified range.
 * @param joint  Which joint (HAA, HFE, KFE) this value represents.
 */
void EnforceLimits(double& q, HyqJointID joint);

Eigen::Vector3d hfe_to_haa_y = Eigen::Vector3d(0.0, -0.083, 0.0); //distance of HFE to HAA in y direction
double hip_offset = 0.083;
double length_thigh = 0.25; // length of upper leg
double length_shank = 0.25; // length of lower leg
double maxDistance = sqrt(pow(hip_offset,2) + pow(length_shank+length_thigh,2)) -0.02;

Eigen::Vector3d ConvertPointToVector(geometry_msgs::Point point);