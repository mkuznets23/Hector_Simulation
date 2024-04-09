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
Vec3<double> ArmPlanner::getPath(int count){
    double pi = 3.14159265359;
    // Circle
    // int speedFactor = 4;
    // double theta = 3.14 * count * 0.001;
    // double R = 0.1; //0.1 works
    // double x = 0.2;
    // // double y = -0.2 + R*cos(theta);
    // double y = R*cos(theta)-0.1; 
    // double z = R* sin(theta);

    // Wave
    double speedFactor = 3;
    double theta1 = pi/4;
    double theta2 = 3*pi/4;
    double theta = sin(speedFactor*count*pi/1000) + (theta1+theta2)/2;
    double R = 0.1; //0.1 works
    double x = 0.2;
    double y = R*cos(theta); 
    double z = R* sin(theta)+0.1;

    // Single Point
    // x = 0.3;
    // y = 0.0;
    // z = 0;

    Vec3<double> P_e;
    P_e << x,y,z;
    return P_e;

}


Vec3<double> ArmPlanner::getPath_waveRight(int count){
    double pi = 3.14159265359;
    // Wave
    double speedFactor = 3;
    double theta1 = pi/4;
    double theta2 = 3*pi/4;
    double theta = sin(speedFactor*count*pi/1000) + (theta1+theta2)/2;
    double R = 0.05; //0.1 works
    double x = 0.1;
    double y = R*cos(theta)-0.15; 
    double z = R*sin(theta);

    Vec3<double> P_e;
    P_e << x,y,z;
    return P_e;
}

Vec3<double> ArmPlanner::getPath_waveLeft(int count){
    double pi = 3.14159265359;
    // Wave
    double speedFactor = 3;
    double theta1 = pi/4;
    double theta2 = 3*pi/4;
    double theta = sin(speedFactor*count*pi/1000) + (theta1+theta2)/2;
    double R = 0.05; //0.1 works
    double x = 0.1;
    double y = -R*cos(theta) + 0.15; 
    double z = R*sin(theta);

    Vec3<double> P_e;
    P_e << x,y,z;
    return P_e;
}

Vec3<double> ArmPlanner::getPath_highFive(int count){
    double s = count * 1.0/1000; // path parameter (in seconds)
    std::cout << "High Five s parameter: " << s << std::endl;
    double x = 0.2;
    if (s < 1){
        x = 0.2+ s*0.1;
    }
    double y = -0.08; 
    double z = 0.1;

    Vec3<double> P_e;
    P_e << x,y,z;
    return P_e;
}

Vec3<double> ArmPlanner::getPath_defaultRight(int count){
    double x = 0.2;
    double y = -0.08; 
    double z = -0.2;

    Vec3<double> P_e;
    P_e << x,y,z;
    return P_e;
}

Vec3<double> ArmPlanner::getPath_defaultLeft(int count){
    double x = 0.2;
    double y = 0.08; 
    double z = -0.2;

    Vec3<double> P_e;
    P_e << x,y,z;
    return P_e;
}

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
        IK(P_e,0)//right
    };

    P_e << 0.2,-0.1,0.1;
    JointPlanElement plan1 = {
        100,
        {0,0,0,0},//left
        IK(P_e,0)//right
    };

    P_e << 0.2,0,0.2;
    JointPlanElement plan2 = {
        200,
        {0,0,0,0},//left
        IK(P_e,0)//right
    };

    P_e << 0.2,0.1,0.1;
    JointPlanElement plan3 = {
        300,
        {0,0,0,0},//left
        IK(P_e,0)//right
    };

    P_e << 0.2,0,0.2;
    JointPlanElement plan4 = {
        400,
        {0,0,0,0},//left
        IK(P_e,0)//right
    };

    P_e << 0.2,-0.1,0.1;
    JointPlanElement plan5 = {
        500,
        {0,0,0,0},//left
        IK(P_e,0)//right
    };

    P_e << 0.2,-0.2,0;
    JointPlanElement plan6 = {
        600,
        {0,0,0,0},//left
        IK(P_e,0)//right
    };

    std::vector<JointPlanElement> plan = {plan0,plan1,plan2,plan3,plan4,plan5,plan6};
    return plan;
}

/*
IK function used to convert an end effector point into joint configuration vector.
'arm' signifies which arm IK is for, 0 is right and 1 is left.
*/
Vec4<double> ArmPlanner::IK(Vec3<double> P_e, int arm){
    double pi = 3.14159265359;
    if (arm == 0){
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

        Vec3<double> P_rotated = Rx*Rz*Ry*P_e;

        std::cout << "P Rotated: " << P_rotated;

        double x = P_rotated[0];
        double y = P_rotated[1];
        double z = P_rotated[2];

        // define joint lengths here for now, but eventually want to pull this from another robot definition
        double L1 = 0.09;//0.094;
        double L2 = 0.2;
        double L3 = 0.2;

        double D = (pow(x,2)+pow(y,2)+pow(z,2)-pow(L1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3);
        double theta4 = atan2(sqrt(1-pow(D,2)),D); // elbow


        double  L = sqrt(pow(x,2)+pow(y,2)-pow(L1,2));
        double theta1 = atan2(y, -x) + atan2(L1, -L); //twist

        double theta2 = atan2(z,sqrt(pow(x,2)+pow(y,2)-pow(L1,2)))-atan2(L3*sin(theta4),L2+L3*cos(theta4)); //shoulder
        double theta3 = 0; // roll - fixed

        theta1 = theta1 - pi/2;
        theta2 = theta2 + pi/2;
        theta4 =  -(theta4 - pi/2); //correct for difference angle 0 between quadruped definition and hector

        Vec4<double> joints;
        joints << theta1, theta2, theta3, theta4;
        return joints;
    }
    else if (arm == 1){
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

        Vec3<double> P_rotated = Rx*Rz*Ry*P_e;

        std::cout << "P Rotated: " << P_rotated;

        double x = P_rotated[0];
        double y = P_rotated[1];
        double z = P_rotated[2];

        // mirror point to right arm side
        double shoulder_width = 0.2;
        y = -y;

        // define joint lengths here for now, but eventually want to pull this from another robot definition
        double L1 = 0.09;//0.094;
        double L2 = 0.2;
        double L3 = 0.2;

        double D = (pow(x,2)+pow(y,2)+pow(z,2)-pow(L1,2)-pow(L2,2)-pow(L3,2))/(2*L2*L3);
        double theta4 = atan2(sqrt(1-pow(D,2)),D); // elbow


        double  L = sqrt(pow(x,2)+pow(y,2)-pow(L1,2));
        double theta1 = atan2(y, -x) + atan2(L1, -L); //twist

        double theta2 = atan2(z,sqrt(pow(x,2)+pow(y,2)-pow(L1,2)))-atan2(L3*sin(theta4),L2+L3*cos(theta4)); //shoulder
        double theta3 = 0; // roll - fixed

        theta1 = theta1 - 3*pi/2;
        theta2 = theta2 + pi/2;
        theta4 =  -(theta4 - pi/2); //correct for difference angle 0 between quadruped definition and hector

        Vec4<double> joints;
        joints << theta1, theta2, theta3, theta4;
        return joints;
    }
    
}