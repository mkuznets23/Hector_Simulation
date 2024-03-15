#include "ArmPlanner.h"
/*
High level controller that outputs arm motion plan as a vector 
of joint configurations with timestamps to follow.
*/

/*
This can return an array of end effector positions for the path. Includes timing information.
Should involve acceleration and deceleration information.
*/
void ArmPlanner::planPath(std::vector<JointPlanElement>& plan){
    
}

/*
IK function will be used to convert an end effector plan into joint configuration plan.
Can implement this later.
*/
void ArmPlanner::IK(){

}