#include "../include/common/cppTypes.h"
#include "ArmController.h"

enum ArmAction{
    HighFive,
    Wave,
    Heart,
    Dance,
    Default
};

class ArmPlanner{
    public:
        ArmPlanner(){};

        // std::vector<JointPlanElement> planPath();
        Vec6<double> getPath_waveRight(int count);
        Vec6<double> getPath_waveLeft(int count);
        Vec6<double> getPath_heart(int count);
        Vec6<double> getPath_heart2(int count);
        Vec6<double> getPath_highFive(int count);
        Vec6<double> getPath_dance(int count, double &roll);
        Vec6<double> getPath_default();
        Eigen::VectorXd IK(Vec6<double> Pe);

        int startCount = 0;
        ArmAction armMovement = Default;
};