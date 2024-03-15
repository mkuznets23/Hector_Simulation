#include "ArmController.h"


/*
This class will manage the execution of an arm plan consiting of joint space information with counter timings.
*/

//Eventually want the plan info to arrive from ArmPlanner, but for now using this to test:
JointPlanElement plan0 = {
    0,
    {0,0,0,0},
    {0,0,0,0}
};
JointPlanElement plan1 = {
    500,
    {0.5,0,0,0},//left
    {0.5,0,0,0}//right
};
JointPlanElement plan2 = {
    1000,
    {-0.5,0,0,0},//left
    {-0.5,0,0,0}//right
};
JointPlanElement plan3 = {
    1500,
    {0,-0.5,0,0},//left
    {0,-0.5,0,0}//right
};
JointPlanElement plan4 = {
    2000,
    {0,1,0.5,0},//left
    {0,1,0.5,0}//right
};
JointPlanElement plan5 = {
    2500,
    {0,1,0.5,3.14},//left
    {0,1,0.5,3.14}//right
};
std::vector<JointPlanElement> testPlan = {plan0,plan1,plan2,plan3,plan4,plan5};

// JointPlanElement plan0 = {
//     0,
//     {0,3.14/2,0,0},
//     {0,3.14/2,0,0}
// };
// JointPlanElement plan1 = {
//     1000,
//     {0,3.14,0,3.14},//left
//     {0,3.14,0,3.14}//right
// };
// std::array<JointPlanElement,2> jointPlan = {plan0,plan1};

std::ofstream plan_log("plan_log.txt");

//c is the FSM counter for when to start executing the path
//this parameter is just for debugging now
//Overall this function should pull in a certain path by reference and point this class to it.
void ArmController::startPath(int c, std::vector<JointPlanElement>& plan){
    // point class path to the one that is passed into this function
    _plan = &plan;

    start_count = c;
    plan_index = 0;
    //turn on flag of path running and point to the path to run <- implementing later
    executing = true;

    //open text file
    plan_log << "starting path" << std::endl;
}

/*
a call to this function in FSM::run() should perform the next move in the path sequence (if the specified count has been reached)
*/
void ArmController::run(ControlFSMData& data){
    if (executing){
        // pull next value in plan array
        int length = std::end(*_plan)-std::begin(*_plan);
        if (plan_index < length){
            auto next = (*_plan)[plan_index];
            int next_time = next.time;
            plan_log << "next = " << next_time << std::endl;
            int current_count_from_start = current_count - start_count;
            plan_log << "current count from start = " << current_count_from_start << std::endl;
            if (current_count_from_start >= next_time){ // if ready to move onto this next step in plan
                plan_index++; //increment plan index to move onto next path element

                // movement
                std::cout << "EXECUTING AT RELATIVE COUNT " << current_count_from_start << std::endl;
                plan_log << "EXECUTING AT RELATIVE COUNT " << current_count_from_start << std::endl;

                //send command to motor to move, based on value of next
                //pretty much set ArmControlCommand in ArmLowLevel object

                Vec4<double> kp(10, 10, 10, 10);
                Vec4<double> kd(1, 1, 1, 1);

                data._armLowLevel->armCommand[0].qDes = next.q_right;
                data._armLowLevel->armCommand[0].kpJoint = kp;
                data._armLowLevel->armCommand[0].kdJoint  = kd;

                data._armLowLevel->armCommand[1].qDes = next.q_left;
                data._armLowLevel->armCommand[1].kpJoint = kp;
                data._armLowLevel->armCommand[1].kdJoint  = kd;
            }
        }
        else{ //plan is finished
            executing = false;
            plan_log.close();
        }
    }
}

/*
Check FSM counter value.
needs to execute before continueExecutePath()
*/
void ArmController::checkCounter(int c)
{
    auto now = std::chrono::system_clock::now();

    auto millis = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    std::cout << "COUNTER: " << c <<  "   Time: " << millis << std::endl;
    plan_log << "COUNTER: " << c <<  "   Time: " << millis << std::endl;
    // pull in counter from FSM walking so it can be accessed by this class
    current_count = c;
}