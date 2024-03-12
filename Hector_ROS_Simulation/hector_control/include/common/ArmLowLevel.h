#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"


struct ArmControlData{
    Vec3<double> q;
}; // represents state of the arms (actual positions of arms, not just motor commands)
    //Also contains index variable for path execution tracking

struct ArmControlCommand{
    ArmControlCommand(){zero();}
    void zero();

    Vec3<double> qDes;
    Vec3<double> kpJoint;
    Vec3<double> kdJoint;
}; // command to give motors

class ArmLowLevel{
    public:
        ArmLowLevel(){
            // for(int i = 0; i < 2; i++){
            //     armCommand[i].zero();
            // }
        };
        ArmControlData armData[2];
        ArmControlCommand armCommand[2];
        void updateCommand(LowlevelCmd* cmd);
};