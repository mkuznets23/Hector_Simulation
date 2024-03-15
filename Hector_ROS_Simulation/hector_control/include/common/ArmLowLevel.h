#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"


struct ArmControlData{
    Vec4<double> q;
}; // represents state of the arms (actual positions of arms, not just motor commands)

struct ArmControlCommand{
    ArmControlCommand(){}
    void zero();

    // ArmControlCommand(){zero();}

    Vec4<double> qDes;
    Vec4<double> kpJoint;
    Vec4<double> kdJoint;
}; // command to give motors

class ArmLowLevel{
    public:
        ArmLowLevel(){
            for(int i = 0; i < 2; i++){
                armCommand[i].zero();
            }
        };
        ArmControlData armData[2];
        ArmControlCommand armCommand[2];
        void updateCommand(LowlevelCmd* cmd);
};