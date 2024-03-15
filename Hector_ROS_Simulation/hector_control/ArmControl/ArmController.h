#include <chrono>
#include "../include/common/ControlFSMData.h"
#include "../include/common/cppTypes.h"

struct JointPlanElement{
    int time; //milliseconds, same as counter in FSM walking
    Vec4<double> q_left;
    Vec4<double> q_right;
    // Twist, Shoulder, Roll, Elbow
};

class ArmController{
    public:
        ArmController(){};
        void checkCounter(int c);
        void startPath(int c);
        void run(ControlFSMData& data);
};