#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHAccel.h>
#include <stdlib.h>
FEHMotor driveR(FEHMotor::Motor3,7.2);
FEHMotor driveL(FEHMotor::Motor2,7.2);
FEHMotor driveF(FEHMotor::Motor0,7.2);
DigitalEncoder shaftR(FEHIO::P0_3);
DigitalEncoder shaftL(FEHIO::P0_4);
DigitalInputPin buttonBR(FEHIO::P0_0);

#define DIAMETER 2.5
#define COUNTS_PER_ROTATION 318.0
#define PI 3.14159265359
#define G_TO_IN 385.827
#define SLOPE_LR 25.0
#define B_LR -2.56
#define SLOPE_F 21.8
#define B_F -2.03
#define POW 80.
#define SLOPE_ROT 0.00847
#define B_ROT -0.042


void moveDist(double, double, double);
void rotate(int ms,double percent);
void moveByTime(double inchX, double inchY, double percent);
void rotateByTime(double revolutions, double percent);

int main() {
    /*
    moveDist(14,14,25);
    moveDist(0,7.75*PI/2.0,25);
    moveDist(10,10,25);
    moveDist(7.75*PI/2.0,0,25);
    moveDist(4,4,25);
    */

    /*Sleep(30.);
    moveByTime(0,9.5,POW);
    rotate(180,80);
    moveByTime(0,10.5,POW);
    moveByTime(8.5,0,POW);
    LCD.SetFontColor(0xFF0000);
    LCD.FillRectangle(0,0,400,400);
    LCD.WriteLine("RED RED RED RED RED RED RED");
    moveByTime(8.5,0,-POW);
    moveByTime(0,9.7,-POW);
    rotate(420,-80);
    moveByTime(0,27.,POW);
    moveByTime(0,25.,-POW);*/

    moveByTime(0,1,POW);
    rotateByTime(0.12,POW);
    moveByTime(1,0,POW);
    /*moveByTime(0,6,POW);
    rotateByTime(0.12,POW);
    moveByTime(0,35,POW);
    moveByTime(0,35,POW);
    rotateByTime(0.45,POW);
    */
}

void rotate(int ms,double percent){
    driveL.SetPercent(-percent);
    driveR.SetPercent(percent);
    driveF.SetPercent(percent);
    Sleep(ms);
    driveF.Stop();
    driveL.Stop();
    driveR.Stop();
}

//temporary thing to just get through the progress check
//requires that percent is at least 20%
void moveByTime(double inchX, double inchY, double percent){
    double tx = inchX / (SLOPE_LR*abs(percent)/100.0 + B_LR);
    double ty = inchY / (SLOPE_F*abs(percent)/100.0 + B_F);
    
    //this is horrible code
    if(tx < ty){
        driveL.SetPercent(-percent);
        driveR.SetPercent(-percent);
        driveF.SetPercent(percent);
        Sleep(tx);
        driveF.Stop();
        Sleep(ty-tx);
        driveL.Stop();
        driveR.Stop();
    }else{
        driveL.SetPercent(-percent); 
        driveR.SetPercent(-percent);
        driveF.SetPercent(percent);
        Sleep(ty);
        driveL.Stop();
        driveR.Stop();
        Sleep(tx-ty);
        driveF.Stop();
    }
}

//temporary thing to just get through the progress check
//requires that percent is at least 20%
void rotateByTime(double revolutions, double percent){
    double sec = revolutions / (SLOPE_ROT*abs(percent) + B_ROT);
    if(sec>0){
        driveL.SetPercent(-percent);
        driveR.SetPercent(percent);
        driveF.SetPercent(percent);
        Sleep(sec);
        driveF.Stop();
        driveL.Stop();
        driveR.Stop();
    }
    
}

void moveDist(double inchL, double inchR,double percent){
    //reset the shaft encoders
    shaftL.ResetCounts();
    shaftR.ResetCounts();
    bool doneL = false;
    bool doneR = false;
    //move the right and left motors until they moved the distance specified
    driveL.SetPercent(percent);
    driveR.SetPercent(percent);
    while(!(doneL&&doneR)){
        LCD.Clear();
        //if the left encoder has had less transitions than needed, move the wheel
        //IF REVISITED LATER, ADD A GRADUAL TRANSITION TO STOPPING
        if(!doneL){
            LCD.WriteLine(shaftL.Counts());
            doneL = shaftL.Counts() > (COUNTS_PER_ROTATION*inchL)/(PI*DIAMETER);
        }else{
            driveL.Stop();
        }
        //if the left encoder has had less transitions than needed, move the wheel
        if(!doneR){
            LCD.WriteLine(shaftR.Counts());
            doneR = shaftR.Counts() > (COUNTS_PER_ROTATION*inchR)/(PI*DIAMETER);
        }else{
            driveR.Stop();
        }    
    }
    driveL.Stop();
    driveR.Stop();

}
