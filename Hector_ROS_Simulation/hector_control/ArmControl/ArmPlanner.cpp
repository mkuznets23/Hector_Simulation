#include "ArmPlanner.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
/*
High level controller that outputs arm motion plan as a vector 
of joint configurations with timestamps to follow.
*/

/*
This can return an array of end effector positions for the path. Includes timing information.
Should already incorporate acceleration and deceleration information.
*/
std::vector<JointPlanElement> ArmPlanner::planPath(){
 
    Vec3<double> P_e;
 
    P_e << 0.2,-0.2,0;
    JointPlanElement plan0 = {
        0,
        {0,0,0,0},//left
        IK(P_e)//right
    };

    P_e << 0.2,-0.1,0.1;
    JointPlanElement plan1 = {
        100,
        {0,0,0,0},//left
        IK(P_e)//right
    };

    P_e << 0.2,0,0.2;
    JointPlanElement plan2 = {
        200,
        {0,0,0,0},//left
        IK(P_e)//right
    };

    P_e << 0.2,0.1,0.1;
    JointPlanElement plan3 = {
        300,
        {0,0,0,0},//left
        IK(P_e)//right
    };

    P_e << 0.2,0,0.2;
    JointPlanElement plan4 = {
        400,
        {0,0,0,0},//left
        IK(P_e)//right
    };

    P_e << 0.2,-0.1,0.1;
    JointPlanElement plan5 = {
        500,
        {0,0,0,0},//left
        IK(P_e)//right
    };

    P_e << 0.2,-0.2,0;
    JointPlanElement plan6 = {
        600,
        {0,0,0,0},//left
        IK(P_e)//right
    };

    // P_e << 0.2,-0.2,0;
    // JointPlanElement plan6 = {
    //     1500,
    //     {0,0,0,0},//left
    //     IK(P_e)//right
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
    //     {0,0,0,0},//left
    //     {0,0,0,0}//right
    // };
    std::vector<JointPlanElement> plan = {plan0,plan1,plan2,plan3,plan4,plan5,plan6};
    return plan;
}

/*
IK function will be used to convert an end effector plan into joint configuration plan.
Can implement this later.
*/
Vec4<double> ArmPlanner::IK(Vec3<double> P_e){
    // this is IK for a quadruped robot limb, so need to apply -pi/2 pitch rotation about arm base to P_e
    double pi = 3.14159265359;
    double th = -pi/2;
    Mat3<double> Rx;
    Rx << 1,0,0,
            0,      cos(-th),   -sin(-th),
            0,sin(-th),cos(-th);
    Mat3<double> Ry;
    Ry << cos(th),0,sin(th),
            0,      1,   0,
            -sin(th),0,cos(th);
    Mat3<double> Rz;
    Rz << cos(th),-sin(th), 0,
            sin(th),  cos(th),  0,
            0,  0,  1;

    Vec3<double> P_rotated = Rx*Rz*Ry*P_e;
    // Vec3<double> P_rotated = Rz*Ry*P_e;

    std::cout << "P Rotated: " << P_rotated;

    double x = P_rotated[0];
    double y = P_rotated[1];
    double z = P_rotated[2];

    // define joint lengths here for now, but eventually want to pull this from another robot definition
    double L1 = 0.094;
    double L2 = 0.2;
    double L3 = 0.2;

    double D = (pow(x,2)+pow(y,2)+pow(z,2)-pow(L1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3);
    double theta4 = atan2(sqrt(1-pow(D,2)),D); // elbow


    double  L = sqrt(pow(x,2)+pow(y,2)-pow(L1,2));
    // double theta1 = -atan2(-y,x) - atan2(L,-L1); // twist
    double theta1 = atan2(y, -x) + atan2(L1, -L); //chat
    // double theta1 = atan2(y*L1+x*L, x*L1-y*L); //Moh

    double theta2 = atan2(z,sqrt(pow(x,2)+pow(y,2)-pow(L1,2)))-atan2(L3*sin(theta4),L2+L3*cos(theta4)); //shoulder
    double theta3 = 0; // roll - fixed

    theta1 = theta1 - pi/2;
    // theta1 = theta1+4.02031;
    // theta1 = theta1 + 0.878722;
    theta2 = theta2 + pi/2;
    theta4 =  -(theta4 - pi/2); //correct for difference angle 0 between quadruped definition and hector

    Vec4<double> joints;
    joints << theta1, theta2, theta3, theta4;
    return joints;
}