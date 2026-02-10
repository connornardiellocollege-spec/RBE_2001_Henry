/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

float destX[4] = {0, 42.9, 84.9, 0};//, 80, 81};
float destY[4] = {0, -43.7, 0, 0};//, 40, 50};
int destNum = 0;

void Robot::UpdatePose(const Twist& twist)
{
    float prevTheta = currPose.theta;
    currPose.theta = currPose.theta + (twist.omega * 0.02)/1.4;
    if(currPose.theta > 3.1415){
        currPose.theta = currPose.theta - 2*3.1415;
    }else if(currPose.theta < -3.1415){
        currPose.theta = currPose.theta + 2*3.1415;
    }
    float coolTheta = (prevTheta + currPose.theta)/2.0;
    currPose.x = currPose.x + twist.u * cos(coolTheta) * 0.02/1.4; 
    currPose.y = currPose.y + twist.u * sin(coolTheta) * 0.02/1.4;
    
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */

#ifdef __NAV_DEBUG__
    // TeleplotPrint("x", currPose.x);
    // TeleplotPrint("y", currPose.y);
    // TeleplotPrint("theta", currPose.theta);
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
    float errorx = destPose.x - currPose.x;
    float errory = destPose.y - currPose.y;
    if(errorx < 3 && errorx > -3 && errory < 3 && errory > -3){
        destNum++;
        if(destNum < sizeof(destX) / sizeof(destX[0])){
            destPose.x = destX[destNum];
            destPose.y = destY[destNum];
            //delay(500);
            retVal = true;
            Serial.print(currPose.x);
            Serial.print(currPose.y);
        }else{
            Serial.print(currPose.x);
            Serial.print(currPose.y);
            robotState = ROBOT_IDLE;
        }
    }
    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        // Serial.print(destPose.x);
        // Serial.print(destPose.y);
        float errorx = destPose.x - currPose.x; //destPose.x;
        float errory = destPose.y - currPose.y; //destPose.y;
        if(errorx < 3 && errorx > -3){
            errorx = 0;
        }
        if(errory < 3 && errory > -3){
            errory = 0;
        } 
        float errorDist = sqrt(errorx * errorx + errory * errory);
        if(errorDist > 30){
            errorDist = 30;
        }
        // TeleplotPrint("errorx", errorx);
        // TeleplotPrint("errory", errory);
        float errortheta = atan2(errory, errorx) - currPose.theta;
        if(errortheta < -3.1415){
            errortheta += 2*3.1415;
        }
        if(errortheta > 3.1415){
            errortheta -= 2*3.1415;
        }
        // TeleplotPrint("errortheta", errortheta);
        float distGain = 4.5;
        float thetaGain = 100.0;
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