#include "ArmPlanner.h"
/*
High level controller that outputs arm motion plan as a vector 
of joint configurations with timestamps to follow.
*/

/*
This can return an array of end effector positions for the path. Includes timing information.
Should already incorporate acceleration and deceleration information.
*/
std::vector<JointPlanElement> ArmPlanner::planPath(){
    JointPlanElement plan0 = {
        0,
        {0.1,0.3,0,0.1},
        {0,0.1,0,0}
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
    std::vector<JointPlanElement> plan = {plan0,plan1,plan2,plan3,plan4,plan5};
    return plan;
}

/*
IK function will be used to convert an end effector plan into joint configuration plan.
Can implement this later.
*/
void ArmPlanner::IK(){
    
}