#include <iostream>
#include <signal.h>
#include "math.h"
#include "unistd.h"

#include "lib/gimbals/Gtracker.hpp"

//ctrl-c 
void ctrlc_handler(sig_atomic_t s){
    printf("\nCaught signal %d\n",s);
    tracker_deinit();
    exit(1);
}

int main(){
    signal(SIGINT,ctrlc_handler);
    tracker_init();

    //set gimbals pole
    BLDCMotor_SetPolePairs(0,11);
    BLDCMotor_SetPolePairs(1,11);

    BLDCMotor_IdleTorque(0,0,2000);
    int lastSpeed;
    float startAngle;

    int motorId=1; //move this gimbal
    int encoderId=0; //encoder connector

    BLDCMotor_IdleTorque(motorId,50,500);

    startAngle=GetPWMEncoderAngle(encoderId,false).fresult;
    while(1){       
        GetInfoResult res=GetPWMEncoderAngle(encoderId,false);
        int speed=360+startAngle-res.fresult;
        speed%=360;
        float fspeed=speed;
        int dir=0;
        if (fspeed>180){
            dir=1;
            fspeed=-fspeed+360;
        }
        fspeed=fspeed*(1-abs(cos(fspeed*3.14/180/2)))/1.8;
        printf("\r Speed %.2f   ",fspeed*(dir?-1:1));
        if (fspeed<0.1)
            BLDCMotor_MoveStop(motorId);
        else
        if(fspeed!=lastSpeed){
            BLDCMotor_MoveContinuous(motorId, dir , 50, fspeed);
        }
        lastSpeed=fspeed;
    }
    return 1;
}