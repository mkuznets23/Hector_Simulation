#include "../include/common/cppTypes.h"
#include "ArmController.h"

class ArmPlanner{
    public:
        ArmPlanner(){};
        Vec3<double> getPath(int count);
        std::vector<JointPlanElement> planPath();
        Vec3<double> getPath_waveRight(int count);
        Vec3<double> getPath_waveLeft(int count);
        Vec4<double> IK(Vec3<double> P_e, int arm);
};