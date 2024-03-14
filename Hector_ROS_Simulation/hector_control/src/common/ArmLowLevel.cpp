#include "../../include/common/ArmLowLevel.h"
/*
This file will use commands to update arm motors
*/
void ArmControlCommand::zero(){
    qDes = Vec4<double>::Zero();
    kpJoint = Vec4<double>::Zero();
    kdJoint = Vec4<double>::Zero();
}

//This function is going to be used to update cmd arm motors much like updateCommand does in LowLevelController.cpp
void ArmLowLevel::updateCommand(LowlevelCmd* cmd){
    //loop that enacts motor commands
    // cmd->motorCmd[11].q = armCommand[1].qDes

    // cmd->motorCmd[11].q = -0.5; 
    cmd->motorCmd[10].q = armCommand[0].qDes[0]; //right twist
    cmd->motorCmd[10].Kp = 10;
    cmd->motorCmd[10].Kd = 1;
    cmd->motorCmd[11].q = armCommand[0].qDes[1]; //right shoulder 
    cmd->motorCmd[11].Kp = 10;
    cmd->motorCmd[11].Kd = 1;
    cmd->motorCmd[12].q = -armCommand[0].qDes[2]; //right roll
    cmd->motorCmd[12].Kp = 10;
    cmd->motorCmd[12].Kd = 1;
    cmd->motorCmd[13].q = armCommand[0].qDes[3]; //right elbow 
    cmd->motorCmd[13].Kp = 10;
    cmd->motorCmd[13].Kd = 1;

    cmd->motorCmd[14].q = armCommand[1].qDes[0]; //left twist
    cmd->motorCmd[14].Kp = 10;
    cmd->motorCmd[14].Kd = 1;
    cmd->motorCmd[15].q = armCommand[1].qDes[1]; //left shoulder
    cmd->motorCmd[15].Kp = 10;
    cmd->motorCmd[15].Kd = 1;
    cmd->motorCmd[16].q = armCommand[1].qDes[2]; //left roll
    cmd->motorCmd[16].Kp = 10;
    cmd->motorCmd[16].Kd = 1;
    cmd->motorCmd[17].q = armCommand[1].qDes[3]; //left elbow
    cmd->motorCmd[17].Kp = 10;
    cmd->motorCmd[17].Kd = 1;

}


//Need some kind of function to be able to keep track of time and arm state though..
//Maybe have the low level stuff about passing to cmd stay in LowLevelController, and this 
//class will update 