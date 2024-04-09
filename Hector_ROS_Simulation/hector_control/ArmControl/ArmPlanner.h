#include "../include/common/cppTypes.h"
#include "ArmController.h"

enum ArmAction{
    HighFive,
    Wave,
    Default
};

class ArmPlanner{
    public:
        ArmPlanner(){};
        Vec3<double> getPath(int count);
        std::vector<JointPlanElement> planPath();
        Vec3<double> getPath_waveRight(int count);
        Vec3<double> getPath_waveLeft(int count);
        Vec3<double> getPath_highFive(int count);
        Vec3<double> getPath_defaultRight(int count);
        Vec3<double> getPath_defaultLeft(int count);
        Vec4<double> IK(Vec3<double> P_e, int arm);

        int highFive_startCount = 0;
        ArmAction armMovement = Default;
};