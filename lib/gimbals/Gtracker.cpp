#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>
#include <iostream>		
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <wiringPi.h>
//#include <wiringPiSPI.h>
#include <stdbool.h>

//#include "timer.h"
#include "Gtracker.hpp"
#include <linux/spi/spidev.h>

#define TEST_COUNT 200


/*
 *          0000 - ustaw pozycje bezwzgledna (0-3600, czyli w stopniach /10 czyli z dokladnoscia do 1/10 stopnia) (pozycja,kierunek i szybkosc)
 *          0001 - ustaw pozycje bezwzgledna automatyczny kierunek (0-3600, czyli w stopniach /10 czyli z dokladnoscia do 1/10 stopnia) (pozycja i szybkosc)
 *          0010 - ustaw pozycje wzgledna (-3600 do 3600, dokladnosc 1/10 stopnia) (pozycja,kierunek i szybkosc)
 *          0011 - ruch ciagly (szybkosc,kierunek)
 *          0100 - ustaw moc silnika w ruchu (0-100)
 *          0101 - ustaw moc i czas utrzymania mocy po ruchu (w spoczynku)
 *          0110 - zatrzymaj silnik
 *          0111 - pobierz i odeslij info
 *          1000 - reboot device - go to bootloader
 *          1001 - odczyt danych z akcelerometru
 *          1010 - zmien domyslny czas przerwania dla regulacji predkosci dzielnikiem
 *          1011 - ustaw liczbe par magnesow 7 dla 12N14P lub 11 dla 24N22P
*/
uchar setAbsPosDirHeader=           0b00000000;
uchar setAbsPosAutoDirHeader=       0b00010000;
uchar setRelPosDirHeader=           0b00100000;
uchar setContMoveHeader=            0b00110000;
uchar setMotorMotionTorqueHeader=   0b01000000;
uchar setMotorIdleTorqueHeader=     0b01010000;
uchar stopMotorHeader=              0b01100000;
uchar getInfoHeader=                0b01110000;
uchar rebootHeader=                 0b10000000;
uchar getAccDataHeader=             0b10010000;
uchar setTimerPeriodHeader=         0b10100000;
uchar setPolesPairCountHeader=      0b10110000;

float angle_x = 0;
float angle_y = 0;

float storedXangle=0;
float storedYangle=0;

float torque0=0;
float torque1=0;

//GIMBAL MOTORS
int motorDriverPin = 17;
static const int CHANNEL = 0;
int fd;

static const uint8_t     spiBPW   = 8 ;
static const uint16_t    spiDelay = 10 ;
static uint32_t    spiSpeeds [2] ;
static int         spiFds [2] ;


void sleepms(int ms){
    usleep(ms*1000);
}

static const char *spi_name = "/dev/spidev0.1";
int wiringPiSPIDataRW2(int channel, unsigned char *data, int len) {
//printf("send [ %x, %x, %x, %x]\n", data[0], data[1], data[2], data[3]);
// As usual, we begin the relationship by establishing a file object which
  //   points to the SPI device.
  int spiDev = open(spi_name, O_RDWR);
  
  // We'll want to configure our SPI hardware before we do anything else. To do
  //   this, we use the ioctl() function. Calls to this function take the form
  //   of a file descriptor, a "command", and a value. The returned value is
  //   always the result of the operation; pass it a pointer to receive a value
  //   requested from the SPI peripheral.
  
  // Start by setting the mode. If we wanted to *get* the mode, we could
  //   use SPI_IOC_RD_MODE instead. In general, the "WR" can be replaced by
  //   "RD" to fetch rather than write. Also note the somewhat awkward
  //   setting a variable rather than passing the constant. *All* data sent
  //   via ioctl() must be passed by reference!
  int mode = 0x01;
  ioctl(spiDev, SPI_IOC_WR_MODE, &mode);
  
  // The maximum speed of the SPI bus can be fetched. You'll find that, on the
  //  pcDuino, it's 12MHz.
  int maxSpeed = 0;
  ioctl(spiDev, SPI_IOC_RD_MAX_SPEED_HZ, &maxSpeed);
  //printf("Max speed: %dHz\n", maxSpeed);
  
  // In rare cases, you may find that a device expects data least significant
  //   bit first; in that case, you'll need to set that up. Writing a 0
  //   indicates MSb first; anything else indicates LSb first.
  int lsb_setting = 0;
  ioctl(spiDev, SPI_IOC_WR_LSB_FIRST, &lsb_setting);
  
  // Some devices may require more than 8 bits of data per transfer word. The
  //   SPI_IOC_WR_BITS_PER_WORD command allows you to change this; the default,
  //   0, corresponds to 8 bits per word.
  int bits_per_word = 0;
  ioctl(spiDev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);



    struct spi_ioc_transfer spi[2];

    channel &= 1;

    // Mentioned in spidev.h but not used in the original kernel documentation
    //	test program )-:

    memset(&spi, 0, sizeof(spi));

    spi[0].tx_buf = (unsigned long long)data;
    spi[0].len = len;
    spi[0].delay_usecs = spiDelay;
    spi[0].speed_hz = 100000;
    spi[0].cs_change=1;
    spi[0].bits_per_word = spiBPW;

    unsigned char dummyData[4] = {0xf0, 0x00, 0x00, 0x00};
    spi[1].tx_buf = (unsigned long long)dummyData;
    spi[1].rx_buf = (unsigned long long)data;
    spi[1].len = len;
    spi[1].cs_change=1;
    spi[1].delay_usecs = spiDelay;
    spi[1].speed_hz = 100000;
    spi[1].bits_per_word = spiBPW;
    int ret=ioctl(spiDev, SPI_IOC_MESSAGE(2), spi);
    close(spiDev);
    return ret;
}
int wiringPiSPIDataW(int channel, unsigned char *data, int len) {
   // printf("send [ %x, %x, %x, %x]\n", data[0], data[1], data[2], data[3]);
// As usual, we begin the relationship by establishing a file object which
  //   points to the SPI device.
  int spiDev = open(spi_name, O_RDWR);
  
  // We'll want to configure our SPI hardware before we do anything else. To do
  //   this, we use the ioctl() function. Calls to this function take the form
  //   of a file descriptor, a "command", and a value. The returned value is
  //   always the result of the operation; pass it a pointer to receive a value
  //   requested from the SPI peripheral.
  
  // Start by setting the mode. If we wanted to *get* the mode, we could
  //   use SPI_IOC_RD_MODE instead. In general, the "WR" can be replaced by
  //   "RD" to fetch rather than write. Also note the somewhat awkward
  //   setting a variable rather than passing the constant. *All* data sent
  //   via ioctl() must be passed by reference!
  int mode = 0x01;
  ioctl(spiDev, SPI_IOC_WR_MODE, &mode);
  
  // The maximum speed of the SPI bus can be fetched. You'll find that, on the
  //  pcDuino, it's 12MHz.
  int maxSpeed = 0;
  ioctl(spiDev, SPI_IOC_RD_MAX_SPEED_HZ, &maxSpeed);
  //printf("Max speed: %dHz\n", maxSpeed);
  
  // In rare cases, you may find that a device expects data least significant
  //   bit first; in that case, you'll need to set that up. Writing a 0
  //   indicates MSb first; anything else indicates LSb first.
  int lsb_setting = 0;
  ioctl(spiDev, SPI_IOC_WR_LSB_FIRST, &lsb_setting);
  
  // Some devices may require more than 8 bits of data per transfer word. The
  //   SPI_IOC_WR_BITS_PER_WORD command allows you to change this; the default,
  //   0, corresponds to 8 bits per word.
  int bits_per_word = 0;
  ioctl(spiDev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);



    struct spi_ioc_transfer spi;

    channel &= 1;

    // Mentioned in spidev.h but not used in the original kernel documentation
    //	test program )-:

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long long)data;
    spi.len = len;
    spi.delay_usecs = spiDelay;
    spi.speed_hz = 100000;
    spi.cs_change=1;
    spi.bits_per_word = spiBPW;


    spi.bits_per_word = spiBPW;


    int ret=ioctl(spiDev, SPI_IOC_MESSAGE(1), &spi);
    close(spiDev);
    return ret;
}

void sendCommand(uchar *buffer, bool rw){
    if (rw)
        wiringPiSPIDataRW2(CHANNEL, buffer, 4);
    else
        wiringPiSPIDataW(CHANNEL, buffer, 4);
}

accData parseAccData(uchar *data){
	accData adata;
    adata.wordL = (short)((((char)data[0] << 8 & 0xFF00) +(char)data[1]) )/64;
    adata.wordH = (short)((((char)data[2] << 8 & 0xFF00) + (char)data[3]) )/64;
    adata.AngleX = adata.wordH;
    adata.AngleY = adata.wordL;
    return adata;
}

accData readAccelerometer(){
    accData adata;
    uchar header=getAccDataHeader;
    uchar cmd[4] = {header, 0x00, 0x00, 0x00};
    sendCommand(cmd, true);

    usleep(500);    
    adata=parseAccData(cmd);
    //printf("acc2 [%d %d | %x, %x, %x, %x]\n", adata.AngleX, adata.AngleY, cmd[0], cmd[1], cmd[2], cmd[3]);
//    printf("\n");
    return adata;
}

//0x30D2 2ms, 0x1869 1ms, [0x0C34 0.5ms], 0x61a 0.25ms, 0x2a 0.01ms, 0xcd 0.034ms
void BLDCMotor_DefaultTimerInterval(short interval){
	uchar intervalMSB = interval >> 8;
	uchar intervalLSB = 0xFF & interval;
	uchar header=setTimerPeriodHeader;
    uchar cmd[4] = {header,intervalMSB, intervalLSB, 0x00};
    sendCommand(cmd,false);
}

//motorId 0-4; angle 0..360.0; direction -1 backward|1 forward|0 auto; torque 0-100; speed 0.01-100;
void BLDCMotor_MoveToAngle(uchar motorId, float angle, short direction, uchar torque, float speed){    
    if (motorId>3) {
        printf("invalid MotorID value (0-3)\n");
        return;
    }
    if (angle<0 or angle>360) {
        printf("invalid angle value (0-360)\n");
        return;
    }
    if (abs(direction)>1) {
        printf("invalid direction value (-1-1)\n");
        return;
    }
    if (torque<0 or torque>100) {
        printf("invalid torque value (0-100)\n");
        return;
    }
    if (speed<0 or speed>100) {
        printf("invalid speed value (0-100)\n");
        return;
    }

    unsigned short zangle,zspeed;
    zangle=angle*10;
    float sp=(speed*4096)/100;
    if (sp-1>0)
        zspeed=sp-1;
    else
        zspeed=1;

    //set the engine power in motion
    uchar header=setMotorMotionTorqueHeader|motorId;
    uchar buf1=torque&0xFF;
    uchar buf2=0&0xFF;
    uchar buf3=0&0xFF;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    if ((motorId==0 && torque0!=torque) || (motorId==1 && torque1!=torque)) //do not send when there is no change
        sendCommand(cmd,false);
    //set angle the motor
    uchar zdirection=0;
    if(direction==1){
        zdirection=0b00001000;
	}
    uchar header2=0;
    if(direction==0)
        header2=setAbsPosAutoDirHeader;
    else
        header2=setAbsPosDirHeader;
    header2|=motorId|zdirection;

    uchar msbspeed=(zspeed>>8)&0x0F;
    uchar msbangle=(zangle>>8)&0x0F;
    msbangle=(msbangle<<4)&0xF0;
    buf1=(msbangle|msbspeed)&0xFF;
    buf2=zangle&0xFF;
    buf3=zspeed&0xFF;
    uchar cmd2[4] = {header2, buf1, buf2, buf3};
    //printf("stp->%x %x %x %x\n",cmd2[0],cmd2[1],cmd2[2],cmd2[3]);
    sendCommand(cmd2,false);
}

//motorId 0-4; angle -360.0..360.0; torque 0-100; speed 0.01-100;
void BLDCMotor_MoveByAngle(uchar motorId, float angle, uchar torque, float speed){
    if (abs(angle)<0.1) //todo in pic
        return;

    if (motorId>3) {
        printf("invalid MotorID value (0-3)\n");
        return;
    }
    if (angle<-360 or angle>360) {
        printf("invalid angle value (0-360)\n");
        return;
    }
    if (torque<0 or torque>100) {
        printf("invalid torque value (0-100)\n");
        return;
    }
    if (speed<0 or speed>100) {
        printf("invalid speed value (0-100)\n");
        return;
    }
    int direction=1; 
    if (angle<0) {
        direction=-1;
        angle=-angle;
    }
    unsigned short zangle,zspeed;
    zangle=angle*10;
    float sp=(speed*4096)/100;
    if (sp-1>0)
        zspeed=sp-1;
    else
        zspeed=1;

    //set the engine power in motion
    uchar header=setMotorMotionTorqueHeader|motorId;
    uchar buf1=torque&0xFF;
    uchar buf2=0&0xFF;
    uchar buf3=0&0xFF;
    uchar cmd[4] = {header, buf1, buf2, buf3};
//    if ((motorId==0 && torque0!=torque) || (motorId==1 && torque1!=torque)) //nie wysyłaj gdy nie ma zmiany    
        sendCommand(cmd,false);

    //move the motor to an angle
    uchar zdirection=0;
    if(direction==1){
        zdirection=0b00001000;
	}
    uchar header2=setRelPosDirHeader|motorId|zdirection;

    uchar msbspeed=(zspeed>>8)&0x0F;
    uchar msbangle=(zangle>>8)&0x0F;
    msbangle=(msbangle<<4)&0xF0;
    buf1=(msbangle|msbspeed)&0xFF;
    buf2=zangle&0xFF;
    buf3=zspeed&0xFF;
    uchar cmd2[4] = {header2, buf1, buf2, buf3};
    //printf("stp->%x %x %x %x\n",cmd2[0],cmd2[1],cmd2[2],cmd2[3]);
    sendCommand(cmd2,false);
    sleepms(2);
}

//motorId 0-4; direction 0 backward|1 forward; torque 0-100; speed 0.05-100;
void BLDCMotor_MoveContinuous(uchar motorId, uchar direction, uchar torque, float speed){
    if (motorId>3) {
        printf("invalid MotorID value (0-3)\n");
        return;
    }
    if (direction>1) {
        printf("invalid direction value (0-1)\n");
        return;
    }
    if (torque<0 or torque>100) {
        printf("invalid torque value (0-100)\n");
        return;
    }
    if (speed<0 or speed>100) {
        printf("invalid speed value (0-100)\n");
        return;
    }

    unsigned short zspeed;
    float sp=(speed*4096)/100;
    if (sp-1>0)
        zspeed=sp-1;
    else
        zspeed=1;

    //set the engine power in motion
    uchar header=setMotorMotionTorqueHeader|motorId;
    uchar buf1=torque&0xFF;
    uchar buf2=0&0xFF;
    uchar buf3=0&0xFF;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    if ((motorId==0 && torque0!=torque) || (motorId==1 && torque1!=torque)) //nie wysyłaj gdy nie ma zmiany
        sendCommand(cmd,false);
    //sleepms(50);
    uchar zdirection=0;
    if(direction==1){
        zdirection=0b00001000;
	}
    uchar header2=setContMoveHeader|motorId|zdirection;

    uchar msbspeed=(zspeed>>8)&0x0F;
    buf1=(msbspeed)&0xFF;
    buf2=0;
    buf3=(zspeed)&0xFF;
    uchar cmd2[4] = {header2, buf1, buf2, buf3};
    //printf("stp->%x %x %x %x\n",cmd2[0],cmd2[1],cmd2[2],cmd2[3]);
    sendCommand(cmd2,false);
}

//motorId 0-4; torque 0-100; idle torque time [0-65535 ms];
void BLDCMotor_IdleTorque(uchar motorId, uchar torque, unsigned short torquems){
    if (motorId>3) {
        printf("invalid MotorID value (0-3)\n");
        return;
    }
    if (torque<0 or torque>100) {
        printf("invalid torque value (0-100)\n");
        return;
    }

    unsigned short zspeed;

    //set engine idle power and duration
    uchar header=setMotorIdleTorqueHeader|motorId;
    uchar buf1=torque&0xFF;
    uchar buf2=(torquems>>8)&0xFF;
    uchar buf3=(torquems)&0xFF;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    sendCommand(cmd,false);
}

//motorId 0-4; polePairs 7 for 12N14P or 11 for 24N22P;
void BLDCMotor_SetPolePairs(uchar motorId, uchar polePairs){
    if (polePairs<=0) {
        printf("invalid polePairs value\n");
        return;
    }
    uchar header=setPolesPairCountHeader|motorId;
    uchar buf1=polePairs&0xFF;
    uchar buf2=0;
    uchar buf3=0;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    sendCommand(cmd,false);
}

//get info (result in worldL): 0 - electrical position; 1 - current Torque; 2 - lastMotionTime - reading of the time since the last movement of the motors;
GetInfoResult BLDCMotor_GetInfo(uchar motorId, uchar info, bool zprint){
    GetInfoResult infoResult;
    if (motorId>3) {
        printf("invalid MotorID value (0-3)\n");
        return infoResult;
    }

    uchar header=getInfoHeader|motorId;
    uchar buf1=info&0xFF;
    uchar buf2=0x00;
    uchar buf3=0x00;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    sendCommand(cmd,true);
	infoResult.wordH = (short)cmd[0] << 8 | cmd[1];
	infoResult.wordL = (short)cmd[2] << 8 | cmd[3];
    infoResult.fresult=(float)infoResult.wordL;
    if (info==0) {
        infoResult.fresult=infoResult.fresult/10;
        infoResult.result=infoResult.fresult;
    }
    if (zprint) {
	    //printf("MotorPos A:%d, B:%d, [%x, %x, %x, %x]\n", infoResult.wordL, infoResult.wordH, cmd[0], cmd[1], cmd[2], cmd[3]);
        if (info==0){
            printf("Angle: %.2f\n",infoResult.fresult);
        }
        if (info==1){
            printf("Torque: %d\n",infoResult.result);
        }
        if (info==2){
            printf("LastMotion: %d\n",infoResult.result);
        }
    }
	return infoResult;
}

//Get gimbal motor position and moving
void BLDCMotor_MoveStop(uchar motorId){
    if (motorId>3) {
        printf("invalid MotorID value (0-3)\n");
        return;
    }
    uchar header=stopMotorHeader|motorId;
    uchar buf1=0;
    uchar buf2=0;
    uchar buf3=0;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    sendCommand(cmd,false);
}

//reset driver
void BLDCMotor_Reset(){
	pinMode(motorDriverPin, OUTPUT);		
	digitalWrite(motorDriverPin, 0);
	digitalWrite(motorDriverPin, 1);
}

//reboot device - go to bootloader
void BLDCMotor_Reboot(){
    uchar header=rebootHeader;
    uchar buf1=0;
    uchar buf2=0;
    uchar buf3=0;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    sendCommand(cmd,false);
}

void tracker_init(){  
    BLDCMotor_IdleTorque(0,0,500);
    BLDCMotor_IdleTorque(1,0,500);
    //angle_x=0;
    //angle_y=BLDCMotor_GetInfo(1,0,true).fresult;
    //BLDCMotor_SetPolePairs(0,11);
    //BLDCMotor_SetPolePairs(1,11);
    //BLDCMotor_IdleTorque(1,10,2000);
    //BLDCMotor_MoveContinuous(1,1,10,20);
    //BLDCMotor_MoveByAngle(1,90,70,30);
    //BLDCMotor_MoveToAngle(1,180,1,70,10);
    //BLDCMotor_Reset();
    //BLDCMotor_GetInfo(1,2,true);
    //BLDCMotor_MoveStop(1);  
    //printf("acc x:%d, y:%d\n",readAccelerometer().AngleX,readAccelerometer().AngleY);
    
    // set the power slowly so they don't jump
    //for (int i=0;i<=30;i++){
    //    BLDCMotor_IdleTorque(0,i,1000);
    //    BLDCMotor_IdleTorque(1,i,1000);        
    //    sleepms(50);
    //}
}

void tracker_deinit(){
    BLDCMotor_MoveStop(0);
    BLDCMotor_MoveStop(1);
    BLDCMotor_IdleTorque(0,0,500);
    BLDCMotor_IdleTorque(1,0,500);
}

float getAngleX(){
    return angle_x;
}
float getAngleY(){
    return angle_y;
}

unsigned short minAngle=250;
unsigned short maxAngle=45388;
float angleFactor=360.0/(maxAngle-minAngle);
//get angle from PWM AS5048A 14-bit magnetic position encoder
GetInfoResult GetPWMEncoderAngle(uchar encoderId, bool zprint){
    GetInfoResult infoResult;
    if (encoderId>1) {
        printf("invalid EncoderID value (0-1)\n");
        return infoResult;
    }

    uchar header=getInfoHeader|encoderId;
    uchar buf1=0x03&0xFF;
    uchar buf2=0x00;
    uchar buf3=0x00;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    sendCommand(cmd,true);
	infoResult.wordH = (short)cmd[0] << 8 | cmd[1];
	infoResult.wordL = (short)cmd[2] << 8 | cmd[3];
    infoResult.fresult=(float)infoResult.wordL;
    unsigned short wh = (unsigned short)cmd[0] << 8 | cmd[1];
    unsigned short wl = (unsigned short)cmd[2] << 8 | cmd[3];
    infoResult.result=infoResult.fresult=(float)(wl-minAngle)*angleFactor;
    if (zprint) {
        if (cmd[0]!=0xff&&cmd[1]!=0xff){
            unsigned short wh = (unsigned short)cmd[0] << 8 | cmd[1];
	        unsigned short wl = (unsigned short)cmd[2] << 8 | cmd[3];
            printf("Encoder read: %u angle:%.4f \n",((unsigned int)wh<<16)|wl,infoResult.fresult);
        }        
    }
	return infoResult;
}
