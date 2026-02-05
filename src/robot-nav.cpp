/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

void Robot::UpdatePose(const Twist& twist)
{
    float prevTheta = currPose.theta;
    currPose.theta = currPose.theta + (twist.omega * 0.02)/1.318;
    float coolTheta = (prevTheta + currPose.theta)/2.0;
    currPose.x = currPose.x + twist.u * cos(coolTheta) * 0.02/1.318; 
    currPose.y = currPose.y + twist.u * sin(coolTheta) * 0.02/1.318;
    
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */

#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta);
#endif

}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    /**
     * TODO: Turn on LED, as well.
     */
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;
    /**
     * TODO: Add code to check if you've reached destination here.
     */

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        float errorx = 40.0 - currPose.x; //destPose.x;
        float errory = 40.0 - currPose.y; //destPose.y;
        if(errorx < 2 && errorx > -2){
            errorx = 0;
        }
        if(errory < 2 && errory > -2){
            errory = 0;
        } 
        float errorDist = sqrt(errorx * errorx + errory * errory);
        TeleplotPrint("errorx", errorx);
        TeleplotPrint("errory", errory);
        float errortheta = atan2(errory, errorx) - currPose.theta;
        float distGain = 4.0;
        float thetaGain = 40.0;
        float vleft = distGain * errorDist - thetaGain * errortheta;
        float vright = distGain * errorDist + thetaGain * errortheta;
        


#ifdef __NAV_DEBUG__
        // Print useful stuff here.
#endif

        chassis.SetMotorEfforts(vleft, vright);
    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */
}