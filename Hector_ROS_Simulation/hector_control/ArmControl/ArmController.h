#include <chrono>
#include "../include/common/ControlFSMData.h"
#include "../include/common/cppTypes.h"

struct JointPlanElement{
    int time;
    Vec3<double> q_left;
    Vec3<double> q_right;
    // Twist, Shoulder, Elbow
};

class ArmController{
    public:
        ArmController(){};
        void checkCounter(int c);
        void startPath(int c);
        void run(ControlFSMData& data);
};