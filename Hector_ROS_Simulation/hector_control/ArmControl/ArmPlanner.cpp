#include "ArmPlanner.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
/*
High level controller that outputs arm motion plan as a vector 
of joint configurations with timestamps to follow.
*/

/*
Get desired end effector position based off of the count variable
*/
Vec6<double> ArmPlanner::getPath_waveRight(int count){
    double pi = 3.14159265359;
    // Left Arm
    double x_l = 0.2;
    double y_l = 0.08; 
    double z_l = -0.2;

    // Right Arm
    double speedFactor = 3;
    double theta1 = pi/4;
    double theta2 = 3*pi/4;
    double theta = sin(speedFactor*count*pi/1000) + (theta1+theta2)/2;
    double R = 0.05;
    double x_r = 0.1;
    double y_r = R*cos(theta)-0.15; 
    double z_r = R*sin(theta);

    Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}

Vec6<double> ArmPlanner::getPath_waveLeft(int count){
    double pi = 3.14159265359;

    // Left Arm
    double speedFactor = 3;
    double theta1 = pi/4;
    double theta2 = 3*pi/4;
    double theta = sin(speedFactor*count*pi/1000) + (theta1+theta2)/2;
    double R = 0.05;
    double x_l = 0.1;
    double y_l = -R*cos(theta) + 0.15; 
    double z_l = R*sin(theta);

    // Right Arm
    double x_r = 0.2;
    double y_r = -0.08; 
    double z_r = -0.2;

    Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}

Vec6<double> ArmPlanner::getPath_highFive(int count){
    // Left Arm
    double x_l = 0.2;
    double y_l = 0.08; 
    double z_l = -0.2;

    // Right Arm
    double s = count * 1.0/1000; // path parameter (in seconds)
    std::cout << "High Five s parameter: " << s << std::endl;
    double x_r = 0.2;
    if (s < 1){
        x_r = 0.2+ s*0.1;
    }
    double y_r = -0.08; 
    double z_r = 0.1;

    Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}

Vec6<double> ArmPlanner::getPath_heart(int count){
    double pi = 3.14159265359;
    double speedFactor = 0.5;    
    double theta = speedFactor * count * pi/1000;
    double R = 0.1; //0.1 works

    //Left Arm 
    double x_l = 0.2;
    double y_l = 0.08; 
    double z_l = -0.2;
    if (theta < pi){
        y_l = R*sqrt(2)*pow(sin(theta),3) - 0.02; 
        z_l = R* (-pow(cos(theta),3)-pow(cos(theta),2) + 2*cos(theta));
    }

    // Right Arm
    double x_r = 0.2;
    double y_r = -0.08; 
    double z_r = -0.2;
    if (theta < pi){
        y_r = -R*sqrt(2)*pow(sin(theta),3) + 0.02; 
        z_r = R* (-pow(cos(theta),3)-pow(cos(theta),2) + 2*cos(theta));
    }

    Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}

Vec6<double> ArmPlanner::getPath_heart2(int count){
    double pi = 3.14159265359;
    double speedFactor = 0.5;    
    double theta = speedFactor * count * pi/1000;
    double R = 0.01; //0.1 works

    //Left Arm 
    double x_l = 0.2;
    double y_l = 0.08; 
    double z_l = -0.2;
    if (theta < pi){
        y_l = R*16*pow(sin(theta),3) - 0.02; 
        z_l = R* (13*cos(theta) - 5 * cos(2*theta) - 2*cos(3*theta)-cos(4*theta));
    }

    // Right Arm
    double x_r = 0.2;
    double y_r = -0.08; 
    double z_r = -0.2;
    if (theta < pi){
        y_r = -R*16*pow(sin(theta),3) + 0.02; 
        z_r = R* (13*cos(theta) - 5 * cos(2*theta) - 2*cos(3*theta)-cos(4*theta));
    }

    Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}

Vec6<double> ArmPlanner::getPath_dance(int count, double &roll){
    double pi = 3.14159265359;
    double speedFactor = 3;
    double theta1 = pi/4;
    double theta2 = 3*pi/4;
    double R = 0.05;

    // Left Arm
    double theta = sin(speedFactor*count*pi/1000) + (theta1+theta2)/2;
    double x_l = 0.1;
    double y_l = -R*cos(theta) + 0.15; 
    double z_l = R*sin(theta);

    // Right Arm
    theta = sin(speedFactor*count*pi/1000) + (theta1+theta2)/2;
    double x_r = 0.1;
    double y_r = R*cos(theta)-0.15; 
    double z_r = R*sin(theta);

    // Roll Behavior
    theta = count * pi/1000;
    roll = 0.2*sin(theta);

     Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}

Vec6<double> ArmPlanner::getPath_prepareBallPickup(){
    // Left Arm
    double x_l = 0.3;
    double y_l = 0.06; 
    double z_l = -0.25;

    // Right Arm
    double x_r = 0.3;
    double y_r = -0.06; 
    double z_r = -0.25;

     Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}

Vec6<double> ArmPlanner::getPath_doBallPickup(int count){
    // Left Arm
    double x_l = 0.3;
    double y_l = 0.04; 
    double z_l = -0.25;

    // Right Arm
    double x_r = 0.3;
    double y_r = -0.04; 
    double z_r = -0.25;

    // if (count > 1000){
    //     // Left Arm
    //     x_l = 0.3;
    //     z_l = -0.1;

    //     // Right Arm
    //     x_r = 0.3;
    //     z_r = -0.1;
    // }

     Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}

Vec6<double> ArmPlanner::getPath_throw(int count){
    double t = count/1000; //in seconds
    double throwTime = 0.2;
    double x_delta = 0.1;
    double z_delta = 0.25;

    // Left Arm
    double x_l = 0.3;
    double y_l = 0.09; 
    double z_l = -0.25;

    // Right Arm
    double x_r = 0.3;
    double y_r = -0.09; 
    double z_r = -0.25;

    if (t < throwTime){
        // Left Arm
        x_l = 0.3 + x_delta * t/throwTime;
        y_l = 0.04; 
        z_l = -0.25 + z_delta * t/throwTime;

        // Right Arm
        x_r = 0.3 + x_delta * t/throwTime;
        y_r = -0.04; 
        z_r = -0.25 + z_delta * t/throwTime;
    }
    else if (t >= throwTime){
        // Left Arm
        x_l = 0.3 + x_delta;
        y_l = 0.04; 
        z_l = -0.25 + z_delta;

        // Right Arm
        x_r = 0.3 + x_delta;
        y_r = -0.04; 
        z_r = -0.25 + z_delta;
    }    

    Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}


Vec6<double> ArmPlanner::getPath_default(){
    // Left Arm
    double x_l = 0.2;
    double y_l = 0.08; 
    double z_l = -0.2;

    // Right Arm
    double x_r = 0.2;
    double y_r = -0.08; 
    double z_r = -0.2;

     Vec6<double> P_e;
    P_e << x_l,y_l,z_l,x_r,y_r,z_r;
    return P_e;
}


/*
This can return an array of end effector positions for the path. Includes timing information.
Should already incorporate acceleration and deceleration information.
*/
// std::vector<JointPlanElement> ArmPlanner::planPath(){
 
//     Vec3<double> P_e;
 
//     P_e << 0.2,-0.2,0;
//     JointPlanElement plan0 = {
//         0,
//         {0,0,0,0},//left
//         IK(P_e,0)//right
//     };

//     P_e << 0.2,-0.1,0.1;
//     JointPlanElement plan1 = {
//         100,
//         {0,0,0,0},//left
//         IK(P_e,0)//right
//     };

//     P_e << 0.2,0,0.2;
//     JointPlanElement plan2 = {
//         200,
//         {0,0,0,0},//left
//         IK(P_e,0)//right
//     };

//     P_e << 0.2,0.1,0.1;
//     JointPlanElement plan3 = {
//         300,
//         {0,0,0,0},//left
//         IK(P_e,0)//right
//     };

//     P_e << 0.2,0,0.2;
//     JointPlanElement plan4 = {
//         400,
//         {0,0,0,0},//left
//         IK(P_e,0)//right
//     };

//     P_e << 0.2,-0.1,0.1;
//     JointPlanElement plan5 = {
//         500,
//         {0,0,0,0},//left
//         IK(P_e,0)//right
//     };

//     P_e << 0.2,-0.2,0;
//     JointPlanElement plan6 = {
//         600,
//         {0,0,0,0},//left
//         IK(P_e,0)//right
//     };

//     std::vector<JointPlanElement> plan = {plan0,plan1,plan2,plan3,plan4,plan5,plan6};
//     return plan;
// }

/*
IK function used to convert an end effector point into joint configuration vector.
'arm' signifies which arm IK is for, 0 is right and 1 is left.
*/
Eigen::VectorXd ArmPlanner::IK(Vec6<double> Pe){
    double pi = 3.14159265359;
    //Right Arm
    // this is IK for a quadruped robot limb, so need to apply -pi/2 pitch rotation about arm base to P_e
        
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
    
    Vec3<double> Pe_R = Pe.segment<3>(3);
    Vec3<double> P_rotated_R = Rx*Rz*Ry*Pe_R;
    std::cout << "P Rotated (Right): " << P_rotated_R;

    double x = P_rotated_R[0];
    double y = P_rotated_R[1];
    double z = P_rotated_R[2];

    // define joint lengths here for now, but eventually want to pull this from another robot definition
    double L1 = 0.09;//0.094;
    double L2 = 0.2;
    double L3 = 0.2;

    double D_R = (pow(x,2)+pow(y,2)+pow(z,2)-pow(L1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3);
    double theta4_R = atan2(sqrt(1-pow(D_R,2)),D_R); // elbow

    double  L_R = sqrt(pow(x,2)+pow(y,2)-pow(L1,2));
    double theta1_R = atan2(y, -x) + atan2(L1, -L_R); //twist

    double theta2_R = atan2(z,sqrt(pow(x,2)+pow(y,2)-pow(L1,2)))-atan2(L3*sin(theta4_R),L2+L3*cos(theta4_R)); //shoulder
    double theta3_R = 0; // roll - fixed

    theta1_R = theta1_R - pi/2;
    theta2_R = theta2_R + pi/2;
    theta4_R =  -(theta4_R- pi/2); //correct for difference angle 0 between quadruped definition and hector

    Vec4<double> joints_R;
    joints_R << theta1_R, theta2_R, theta3_R, theta4_R;

    // Left Arm
    // this is IK for a quadruped robot limb, so need to apply -pi/2 pitch rotation about arm base to P_e
    Vec3<double> Pe_L = Pe.segment<3>(0);
    Vec3<double> P_rotated_L = Rx*Rz*Ry*Pe_L;
    std::cout << "P Rotated (Left): " << P_rotated_L;

    x = P_rotated_L[0];
    y = P_rotated_L[1];
    z = P_rotated_L[2];

    // mirror point to right arm side
    y = -y;
    
    double D_L = (pow(x,2)+pow(y,2)+pow(z,2)-pow(L1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3);
    double theta4_L = atan2(sqrt(1-pow(D_L,2)),D_L); // elbow


    double  L_L = sqrt(pow(x,2)+pow(y,2)-pow(L1,2));
    double theta1_L = atan2(y, -x) + atan2(L1, -L_L); //twist

    double theta2_L = atan2(z,sqrt(pow(x,2)+pow(y,2)-pow(L1,2)))-atan2(L3*sin(theta4_L),L2+L3*cos(theta4_L)); //shoulder
    double theta3_L = 0; // roll - fixed

    theta1_L = theta1_L - 3*pi/2;
    theta2_L = theta2_L + pi/2;
    theta4_L =  -(theta4_L - pi/2); //correct for difference angle 0 between quadruped definition and hector

    Vec4<double> joints_L;
    joints_L << theta1_L, theta2_L, theta3_L, theta4_L;

    Eigen::VectorXd joints(8);
    joints << joints_L, joints_R;
    
    return joints;
    
}