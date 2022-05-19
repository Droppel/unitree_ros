/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>

namespace unitree_model {

ros::Publisher servo_pub[12];
unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 300;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 300;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 3;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 3;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = lowState.motorState[i].q;
    }
}

void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 2*1000);
}

void moveTowr(xpp::Joints joints, std::vector<geometry_msgs::Vector3> forces, double duration, int counter)
{   
    double* pos = new double[12];
    for (int i = 0; i < 12; i++) {
        pos[i] = joints.GetJoint(i);
    }
    newMoveAllPosition(forces, pos, duration, counter);
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd(double sleepAfter)
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd(1);
    }
}

void newMoveAllPosition(std::vector<geometry_msgs::Vector3> forces, double* targetPos, double duration, int counter)
{
    limbFTtoJointTorque(0, targetPos, forces);
    limbFTtoJointTorque(1, targetPos, forces);
    limbFTtoJointTorque(2, targetPos, forces);
    limbFTtoJointTorque(3, targetPos, forces);
    for(int j=0; j<12; j++){
        lowCmd.motorCmd[j].q = targetPos[j]; 
    }
    sendServoCmd(duration);
    // for (int j=0; j<12; j++) {
    //     ROS_INFO_STREAM(std::fixed << "Step: " << counter << "Joint: " << j << "\n" << lowCmd.motorCmd[j]);
    // } 
}

//Needs the NEXT sensormsg
void getJacobian(int limb, Eigen::MatrixXd& J, double* targetPos)
{
    double q1=0,q2=0,q3=0,q4=0,q5=0,q6=0;
    double s1,s2,s3,s4,s5,s6;
    double c1,c2,c3,c4,c5,c6;
    q1= targetPos[3 * limb];
    q2= targetPos[3 * limb + 1];
    q3= targetPos[3 * limb + 2];
    s1 = sin(q1);s2 = sin(q2);s3 = sin(q3);s4 = sin(q4);s5 = sin(q5);s6 = sin(q6);
    c1 = cos(q1);c2 = cos(q2);c3 = cos(q3);c4 = cos(q4);c5 = cos(q5);c6 = cos(q6);
    J.resize(0,0);
    J.resize(3,3);
    J <<    0,                                                     -0.25*c2*c3 - 0.25*c2 + 0.25*s2*s3,          -0.25*c2*c3 + 0.25*s2*s3, 
            0.25*c1*c2*c3 + 0.25*c1*c2 - 0.25*c1*s2*s3 - 0.083*s1, -0.25*c2*s1*s3 - 0.25*c3*s1*s2 - 0.25*s1*s2, -0.25*c2*s1*s3 - 0.25*c3*s1*s2, 
            0.083*c1 + 0.25*c2*c3*s1 + 0.25*c2*s1 - 0.25*s1*s2*s3,  0.25*c1*c2*s3 + 0.25*c1*c3*s2 + 0.25*c1*s2,  0.25*c1*c2*s3 + 0.25*c1*c3*s2;
//                     0, c1, s1*s2,
//                     1, 0, c2,
//                     0, -s1, s2*c1;

}

void limbFTtoJointTorque(int limb, double* targetPos, std::vector<geometry_msgs::Vector3> forces)
{
    Eigen::MatrixXd J,T,limbFT;
        getJacobian(limb, J, targetPos);
        limbFT.resize(0,0);
        limbFT.resize(3,1);
        // limbFT <<   jointLimbFT[limb].force.x,
        //             jointLimbFT[limb].force.y,
        //             jointLimbFT[limb].force.z,
        //             jointLimbFT[limb].torque.x,
        //             jointLimbFT[limb].torque.y,
        //             jointLimbFT[limb].torque.z;
        limbFT << forces[limb].x,
                  forces[limb].y,
                  forces[limb].z;
                //   0,
                //   0,
                //   0;
        T = J.transpose()*limbFT;
        setLimbTorques(limb, T, targetPos);
 }

 void setLimbTorques(int limb, Eigen::MatrixXd T, double* targetPos) {
    double pMin = 10;
    double pMax = 5000;

    double errorQ0 = lowState.motorState[3 * limb].q - targetPos[3 * limb];
    double errorQ1 = lowState.motorState[3 * limb + 1].q - targetPos[3 * limb + 1];
    double errorQ2 = lowState.motorState[3 * limb + 2].q - targetPos[3 * limb + 2];

    double pgain0 = abs(T(0) / errorQ0);
    double pgain1 = abs(T(0) / errorQ1);
    double pgain2 = abs(T(0) / errorQ2);


    pgain0 = coerce(pgain0, pMin, pMax);
    pgain1 = coerce(pgain1, pMin, pMax);
    pgain2 = coerce(pgain2, pMin, pMax);

    lowCmd.motorCmd[3 * limb].Kp = pgain0;
    lowCmd.motorCmd[3 * limb + 1].Kp = pgain1;
    lowCmd.motorCmd[3 * limb + 2].Kp = pgain2;
 }

 double coerce(double x, double min, double max)
	{
		// Coerce the value x to be in the range [min,max]
		if(min > max) return 0.5*(min + max);
		else if(x >= max) return max;
		else if(x <= min) return min;
		else return x;
	}
}
