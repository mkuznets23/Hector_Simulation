#include "../include/common/cppTypes.h"
#include "ArmController.h"

class ArmPlanner{
    public:
        ArmPlanner(){};
        std::vector<JointPlanElement> planPath();
        void IK();
};