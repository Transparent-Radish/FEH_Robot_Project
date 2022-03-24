#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <stdlib.h>

FEHMotor driveR(FEHMotor::Motor1,7.2);
FEHMotor driveL(FEHMotor::Motor2,7.2);
FEHMotor driveF(FEHMotor::Motor3,7.2);
FEHMotor arm(FEHMotor::Motor0,5);

AnalogInputPin cds(FEHIO::P2_7);
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
#define POW_SLOW 30.
#define POW 80.
#define SLOPE_ROT 0.00847
#define B_ROT -0.042
#define RED_THRESHOLD 1.5
#define RPS_DELAY 0.35
#define H_RANGE 5

void moveDist(double, double, double);
void rotate(int ms,double percent);
void moveByTime(double inchX, double inchY, double percent);
void rotateByTime(double revolutions, double percent);
void burgerFlip(double inchX,double inchY,double wheelPercent, double armPercent, int armTime);
void go(double,double,double,double);
int iceCreamFlavor,x,y,heading;



int main() {

    RPS.InitializeTouchMenu();
    while(cds.Value()>RED_THRESHOLD){}
    iceCreamFlavor = RPS.GetIceCream();

    //align robot towards ramp and move up
    moveByTime(0,13,POW);
    rotateByTime(43,POW);
    moveByTime(0,25,POW);
    //update position values
    x = RPS.X();
    y = RPS.Y();
    heading = RPS.Heading();
    Sleep(RPS_DELAY);


    //go to the top of the ramp
    go(18,50,270,POW);

    //go to the chocl

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

    /*
    
    //wait for light to turn on
    while(cds.Value()>RED_THRESHOLD){}

    //align robot towards ramp and move up
    moveByTime(0,11,POW);
    rotateByTime(0.115,POW);
    moveByTime(0,40,POW);

    //align robot with sink and deposit tray
    rotateByTime(0.5,-POW_BEFORE_SINK);
    moveByTime(20,0,POW_BEFORE_SINK);
    moveByTime(0,7,POW_BEFORE_SINK);
    arm.SetPercent(60);
    Sleep(1500);
    arm.SetPercent(-60);
    Sleep(1500);
    arm.Stop();
    moveByTime(5,0,-POW_BEFORE_SINK);
    moveByTime(0,5,-POW_BEFORE_SINK);
    rotateByTime(0.25,-POW_BEFORE_SINK);   
    moveByTime(0,22.5,POW_BEFORE_SINK);
    rotateByTime(0.25,-POW_BEFORE_SINK);
    moveByTime(0,23,-POW_BEFORE_SINK);
    moveByTime(7,0,-POW_BEFORE_SINK);
    moveByTime(0,22,POW_BEFORE_SINK);
    moveByTime(0,24,-POW_BEFORE_SINK);
    moveByTime(7,0,-POW_BEFORE_SINK);
    moveByTime(0,24,POW_BEFORE_SINK);
*/
/*while(cds.Value()>RED_THRESHOLD){}
    moveByTime(0,11,POW);
    rotateByTime(0.115,POW);
    moveByTime(0,48,POW);
    moveByTime(10,0,POW_SLOW);
    moveByTime(0,7,POW_SLOW);
    moveByTime(0,2,-POW_SLOW);
    moveByTime(9,0,-POW_SLOW);
    burgerFlip(10,5.0,POW_SLOW, 100, 1000);*/

    /*
    rotateByTime(0.25,-POW_BEFORE_SINK);    
    moveByTime(0,22.5,POW_BEFORE_SINK);
    moveByTime(5,0,-POW_BEFORE_SINK);
    rotateByTime(0.25,-POW_BEFORE_SINK);

    moveByTime(0,10,-POW_BEFORE_SINK);
    moveByTime(7,0,-POW_BEFORE_SINK);
    moveByTime(0,22,POW_BEFORE_SINK);
    */

    /*moveByTime(0,6,POW);
    rotateByTime(0.12,POW);
    moveByTime(0,35,POW);
    moveByTime(0,35,POW);
    rotateByTime(0.45,POW);
    */
}

void rotate(int ms,double percent){
    driveL.SetPercent(percent);
    driveR.SetPercent(percent);
    driveF.SetPercent(-percent);
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
    if(ty > tx){
        driveL.SetPercent(percent);
        driveR.SetPercent(-percent);
        driveF.SetPercent(percent);
        Sleep(tx);
        driveF.Stop();
        Sleep(ty-tx);
        driveL.Stop();
        driveR.Stop();
    }else{
        driveL.SetPercent(percent); 
        driveR.SetPercent(-percent);
        driveF.SetPercent(percent);
        Sleep(ty);
        driveL.Stop();
        driveR.Stop();
        Sleep(tx-ty);
        driveF.Stop();
    }
}

void burgerFlip(double inchX,double inchY,double wheelPercent, double armPercent, int armTime){

    
        driveR.SetPercent(-25);

    //move arm up
    arm.SetPercent(-armPercent);
    Sleep(armTime);
    arm.Stop();
    //move to the right
        driveR.SetPercent(-25);

    moveByTime(inchX,0,wheelPercent);

    //move back    
    driveR.SetPercent(-25);

    moveByTime(0,inchY,-wheelPercent);

    //move further to the right
        driveR.SetPercent(-25);

    moveByTime(inchX/2.0,0,wheelPercent);

    //move forward
        driveR.SetPercent(-25);

    moveByTime(0,inchY,wheelPercent);

    //move left
        driveR.SetPercent(-25);

    moveByTime(1.5*inchX,0,-wheelPercent);

    //lower arm
    arm.SetPercent(armPercent);
    Sleep(armTime);
    arm.Stop();
}

//temporary thing to just get through the progress check
//requires that percent is at least 20%
void rotateByTime(double degrees, double percent){
    double revolutions = degrees/360.0;
    double sec = revolutions / (SLOPE_ROT*abs(percent) + B_ROT);
    if(sec>0){
        driveL.SetPercent(-percent);
        driveR.SetPercent(-percent);
        driveF.SetPercent(-percent);
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

void go(double x_,double y_,double heading_,double power){
    //update position values
    if(RPS.X() >= 0)
        x = RPS.X();
    if(RPS.Y() >= 0)
        y = RPS.Y();
    if(RPS.Heading() >= 0)
        heading = RPS.Heading();

    Sleep(RPS_DELAY);

    moveByTime(x_ - x,y_ - y,power);

    //if the angle is way out of wack, change it
    if(heading < heading_ - H_RANGE/2.0 || heading > heading_ + H_RANGE/2.0){
        rotateByTime(heading - heading_,power);
    }

    if(RPS.X() >= 0)
        x = RPS.X();
    else
        x = x_;

    if(RPS.Y() >= 0)
        y = RPS.Y();
    else
        y = y_;

    if(RPS.Heading() >= 0)
        heading = RPS.Heading();
    else
        heading = heading_;
    
    Sleep(RPS_DELAY);

    
}
