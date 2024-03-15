#include <chrono>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
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
        void startPath(int c, std::vector<JointPlanElement>& plan);
        void run(ControlFSMData& data);

        std::vector<JointPlanElement> *_plan;
        int start_count = 0;
        int current_count = 0;
        int plan_index = 0;
        bool executing = false;
        
};