#include "../include/common/cppTypes.h"
#include "ArmController.h"

class ArmPlanner{
    public:
        ArmPlanner(){};
        std::vector<JointPlanElement> planPath();
        Vec4<double> IK(Vec3<double> P_e);
};