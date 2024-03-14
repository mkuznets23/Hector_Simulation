#include "ArmController.h"
#include <iostream>
#include <fstream>
#include <map>

/*
This will contain a class that manages the arm data and commands:
-Path planning function will determine high level points to take the end effector at which times.
    (for this to work, need to keep track of time passing)

-IK function will calculate motor positions to command
-Update function will continuously set the arms to the commanded positions
-Data returning function will return and allow updating the state data of the arms
    (necessary since the commands may not reach targets exactly)

In .h file the constructor can have a variable for planning
*/

// long long time; //time variable that gets used to keep track of path execution.
    //Need to initialize time to 0 when this class first gets intantiated, then this will be used as a reference
    //for all arm movement.

JointPlanElement plan0 = {
    0,
    {0,0,0,0},
    {0,0,0,0}
};

JointPlanElement plan1 = {
    1000,
    {0,0.5,0.5,0},//left
    {0,0.5,0.5,0}//right
};

// JointPlanElement plan1 = {
//     1000,
//     {0,0,0.5,0},
//     {0,0,0.5,0}
// };

JointPlanElement plan2 = {
    2000,
    {0,-0.5,0.5,3.14},//left
    {0,-0.5,0.5,3.14}//right
};

// JointPlanElement plan3 = {
//     2000,
//    {0,0,-0.5,3.14},
//     {0,0,-0.5,3.14}
// };

std::array<JointPlanElement,3> jointPlan = {plan0,plan1,plan2};
// std::array<JointPlanElement,2> jointPlan = {plan0,plan1};


// std::map<int, int> testPlanMap;
//for now testPlan is fine, but later path should be a class instance field
int start_count = 0;
int current_count = 0;
int plan_index = 0;
bool executing = false;
std::ofstream plan_log("plan_log.txt");
/*
This function needs to set commands to the ArmControlCommand struct as requests to move.
It does this in a way coordinated with time info provided from path plan array.
Can store an index of which path plan array step the program is currently at.
(This index variable can be in ArmControlData struct)
Index variable is actively incrementing at a set dt.
These variables will be initially created when the main function initializes, so will persist through each FSM run iteration.

Modifications to be made in FSM::run()
While any more path array is left for execution (index not at end), it needs to be included in the looping FSM::run() function,
where the executePath can increment at whatever rate in needs to based on specified timings

*/

// void convertToCounterScale(int (&plan)[]){
    
// }

//c is the FSM counter for when to start executing the path
//this parameter is just for debugging control now
void ArmController::startPath(int c){
    // set start time to whatever the system time is

    // auto now = std::chrono::system_clock::now();
    // start_time = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    start_count = c;
    plan_index = 0;
    //turn on flag of path running and point to the path to run
    executing = true;

    //open text file
    plan_log << "starting path" << std::endl;
}

// a call to this function in FSM::run() should perform the next move in the path sequence (if the specified dt has passed)
// this way the freqeuncy of FSM::run and time resolution of path dont need to match. -> avoid errors down the line
void ArmController::run(ControlFSMData& data){
    // check counter 

    // if count is equal to next path element, execute move.

    if (executing){
        // pull next value in plan array
        int length = std::end(jointPlan)-std::begin(jointPlan);
        if (plan_index < length){
            auto next = jointPlan[plan_index];
            int next_time = next.time;
            plan_log << "next = " << next_time << std::endl;
            int current_count_from_start = current_count - start_count;
            plan_log << "current count from start = " << current_count_from_start << std::endl;
            if (current_count_from_start >= next_time){ // if ready to move onto this next step in plan
                plan_index++; //increment plan index to move onto next path element

                // do something with next (eventually actually do the move)
                std::cout << "EXECUTING AT RELATIVE COUNT " << current_count_from_start << std::endl;
                plan_log << "EXECUTING AT RELATIVE COUNT " << current_count_from_start << std::endl;

                //send command to motor to move, based on value of next
                //pretty much set ArmControlCommand in ArmLowLevel object

                Vec4<double> kp(10, 10, 10, 10);
                Vec4<double> kd(1, 1, 1, 1);
                // data._armLowLevel->armCommand[0].qDes = next.q_right;
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

//Check FSM counter value.
//needs to execute before continueExecutePath()
void ArmController::checkCounter(int c)
{
    auto now = std::chrono::system_clock::now();
    // auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    auto millis = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    std::cout << "COUNTER: " << c <<  "   Time: " << millis << std::endl;
    plan_log << "COUNTER: " << c <<  "   Time: " << millis << std::endl;
    // pull in counter from FSM walking so it can be accessed by this class
    current_count = c;
}