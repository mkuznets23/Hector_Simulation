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
        void startPath(int c, std::vector<JointPlanElement>* plan);
        void run(ControlFSMData& data);
        void set(ControlFSMData& data, Vec4<double> jointDes_R, Vec4<double> jointDes_L);

        // JointPlanElement plan0 = {
        //     0,
        //     {0,0,0,0},
        //     {0,0,0,0}
        // };
        // JointPlanElement plan1 = {
        //     500,
        //     {0.5,0,0,0},//left
        //     {0.5,0,0,0}//right
        // };
        // JointPlanElement plan2 = {
        //     1000,
        //     {-0.5,0,0,0},//left
        //     {-0.5,0,0,0}//right
        // };
        // JointPlanElement plan3 = {
        //     1500,
        //     {0,-0.5,0,0},//left
        //     {0,-0.5,0,0}//right
        // };
        // JointPlanElement plan4 = {
        //     2000,
        //     {0,1,0.5,0},//left
        //     {0,1,0.5,0}//right
        // };
        // JointPlanElement plan5 = {
        //     2500,
        //     {0,1,0.5,3.14},//left
        //     {0,1,0.5,3.14}//right
        // };
        // std::vector<JointPlanElement> testPlan = {plan0,plan1,plan2,plan3,plan4,plan5};

        std::vector<JointPlanElement>* _plan = nullptr;
        int start_count = 0;
        int current_count = 0;
        int plan_index = 0;
        bool executing = false;
        
};