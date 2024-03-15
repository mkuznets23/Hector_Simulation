#include "../include/common/cppTypes.h"
#include "ArmController.h"

class ArmPlanner{
    public:
        ArmPlanner(){};
        void planPath(std::vector<JointPlanElement>& plan);
        void IK();
};