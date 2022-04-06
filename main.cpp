#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHSD.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

FEHMotor driveR(FEHMotor::Motor1,7.2);
FEHMotor driveL(FEHMotor::Motor2,7.2);
FEHMotor driveF(FEHMotor::Motor3,7.2);
FEHMotor arm(FEHMotor::Motor0,5);

AnalogInputPin cds(FEHIO::P2_7);
DigitalEncoder shaftR(FEHIO::P0_3);
DigitalEncoder shaftL(FEHIO::P0_4);
DigitalInputPin buttonR(FEHIO::P2_6);
DigitalInputPin buttonL(FEHIO::P2_5);

FEHServo frontArm(FEHServo::Servo0);


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
#define RPS_DELAY 0.8
#define H_RANGE 5
#define ROTATION_CORRECTION_FACTOR 0.93
#define ARM_MIN 1000
#define ARM_MAX 1800

void moveDist(double, double, double);
void rotate(int ms,double percent);
void moveByTime(double inchX, double inchY, double percent);
void rotateByTime(double revolutions, double percent);
void burgerFlip(double inchX,double inchY,double wheelPercent);
void goToLocation(double xNew,double yNew,double headingNew,double power,char*);
int iceCreamFlavor;
double x,y,heading;
void moveForwardUntilBump(double percent);
void generateLocationValues(double* x, double* y, double* heading);
void goToLocation2(double xNew,double yNew,double headingNew,double power,char* name);
void recordTheDarnLocationsInRPS();
int main() {



    RPS.InitializeTouchMenu();

    recordTheDarnLocationsInRPS();

    /*double locationCoords[3][3];
    char* places[] = {"RAMPBOT.txt", "RAMPTOP.txt", "SINK.txt"};

    //get rps coords of different places
    for(int i=0;i<3;i++){
        LCD.Clear();
        LCD.WriteLine(places[i]);
        generateLocationValues(&locationCoords[i][0],&locationCoords[i][1],&locationCoords[i][2]);
    }
    */
    LCD.Clear();
    LCD.WriteLine("FINAL ACTION");
    float a,b;
    //wait until the user touches the screen and releases
    LCD.ClearBuffer();
    while(!LCD.Touch(&a,&b)){}
    while(LCD.Touch(&a,&b)){}
    LCD.Clear();
    LCD.WriteLine("READY IN 30 SECONDS OR WHEN THE LIGHT TURNS ON");

    

    double startTime = TimeNow();
    while(cds.Value()>RED_THRESHOLD && TimeNow() - startTime < 28.0){}
    moveByTime(0,10,POW);
    rotateByTime(-45,POW);
    moveByTime(0,40,90);
    
    x = RPS.X();
    y = RPS.Y();
    heading = RPS.Heading();
    
    //move the damn robot
    for(int i=0;i<3;i++){
        //goToLocation2(locationCoords[i][0],locationCoords[i][1],locationCoords[i][2], POW,places[i]);
    }
    


    
    //update position values
    //x = RPS.X();
    //y = RPS.Y();
    //heading = RPS.Heading();
    //Sleep(RPS_DELAY);

    /*frontArm.SetMin(ARM_MIN);
    frontArm.SetMax(ARM_MAX);
    
    frontArm.SetDegree(0);
    RPS.InitializeTouchMenu();

    LCD.WriteLine("FINAL ACTION");
    float a,b;
    //wait until the user touches the screen and releases
    LCD.ClearBuffer();
    while(!LCD.Touch(&a,&b)){}
    while(LCD.Touch(&a,&b)){}
    LCD.WriteLine("INITIATED");


    frontArm.SetMin(ARM_MIN);
    frontArm.SetMax(ARM_MAX);
    frontArm.SetDegree(0);
    
    double startTime = TimeNow();
    while(cds.Value()>RED_THRESHOLD && TimeNow() - startTime < 28.0){}
    iceCreamFlavor = 1;//RPS.GetIceCream();

    //go to front of ramp
    //goToLocation(13,23,0, POW_SLOW);

    //go to the top of the ramp
    //goToLocation(13,50,0, POW_SLOW);

    //go to the chocolate lever
    //goToLocation(19,60,45, POW_SLOW);

    //SD.FClose(testLog);

    moveByTime(0,10,POW);
    rotateByTime(45,POW);
    moveByTime(0,40,90);
    moveByTime(5,0,-POW);
    moveForwardUntilBump(POW_SLOW);
    moveByTime(0,7,-POW_SLOW);
    rotateByTime(40,-POW);
    moveByTime(5,0,POW_SLOW);
    switch(iceCreamFlavor){
        case 0:
            moveByTime(8,0,-POW_SLOW);
        break;    
        case 1:
            moveByTime(4,0,-POW_SLOW);
        break;
    }
    moveByTime(0,3,POW_SLOW);
    frontArm.SetDegree(80);
    Sleep(500);
    frontArm.SetDegree(0);
    moveByTime(0,6,-POW_SLOW);
    Sleep(5.0);
    frontArm.SetDegree(150);
    Sleep(2.0);
    moveByTime(0,7,POW_SLOW);
    frontArm.SetDegree(65);

    moveByTime(0,7,-POW);
    frontArm.SetDegree(0);
    rotateByTime(135,-POW);
    switch(iceCreamFlavor){
        case 0:
            moveByTime(18,8.5,POW);
        break;    
        case 1:
            moveByTime(16,7,POW);
        break;
        case 2:
            moveByTime(12,5,POW);
        break;
    }    
    moveForwardUntilBump(POW_SLOW);
    moveByTime(16,0,-POW);
    moveByTime(0,20,POW);
    rotateByTime(50,-POW);
    moveByTime(0,50,POW);
    */

    /*while(true){
        x = RPS.X();
        y = RPS.Y();
        heading = RPS.Heading();
        Sleep(RPS_DELAY);
        LCD.Clear();
        LCD.WriteLine(heading);
    }*/

    

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
    frontArm.SetMin(ARM_MIN);
    frontArm.SetMax(ARM_MAX);
    
    frontArm.SetDegree(0);
    RPS.InitializeTouchMenu();

    LCD.WriteLine("FINAL ACTION");
    float a,b;
    //wait until the user touches the screen and releases
    LCD.ClearBuffer();
    while(!LCD.Touch(&a,&b)){}
    while(LCD.Touch(&a,&b)){}

    //wait for light to turn on

    double startTime = TimeNow();
    while(cds.Value()>RED_THRESHOLD && TimeNow() - startTime < 28.0){}

    //align robot towards ramp and move up
    moveByTime(0,10,POW);
    rotateByTime(45,POW);
    moveByTime(0,50,POW);

    //align robot with sink and deposit tray
    rotateByTime(180,POW_SLOW);
    moveByTime(18,0,POW_SLOW);
    moveForwardUntilBump(POW_SLOW);
    /*arm.SetPercent(60);
    Sleep(1500);
    arm.SetPercent(-60);
    Sleep(1500);
    arm.Stop();
    */
    /*
    frontArm.SetDegree(180);
    Sleep(2000);
    frontArm.SetDegree(0);

    //go to the burger station
    moveByTime(18,8,-POW);
    rotateByTime(180,POW);
    moveByTime(10,10,POW);
    moveForwardUntilBump(POW);
    moveByTime(10,5,-POW);
    frontArm.SetDegree(180);
    moveByTime(5,0,POW);
    burgerFlip(10,5,POW_SLOW);

    //line up with burger station and move to ticket
    moveForwardUntilBump(POW_SLOW);
    moveByTime(5,0,POW);
    arm.SetPercent(60);
    Sleep(1500);
    moveByTime(0,18,-POW);
    moveByTime(15,0,-POW);

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

void moveForwardUntilBump(double percent){
    driveL.SetPercent(percent);
    driveR.SetPercent(-percent);
    double startTime = TimeNow();
    while((buttonL.Value()||buttonR.Value()) && TimeNow() - startTime < 15.0){}
    driveL.Stop();
    driveR.Stop();
}

//temporary thing to just get through the progress check
//requires that percent is at least 20%
void moveByTime(double inchX, double inchY, double percent){
    double tx = inchX / (SLOPE_LR*abs(percent)/100.0 + B_LR);
    double ty = inchY / (SLOPE_F*abs(percent)/100.0 + B_F);
    int xSign = 1, ySign = 1;
    if(inchX < 0){
        xSign = -1;
        tx *= -1;
    }
    if(inchY < 0){
        ySign = -1;
        ty *= -1;
    }
        

    //this is horrible code
    if(ty > tx){
        /*driveL.SetPercent(ySign*percent);
        driveR.SetPercent(-ySign*percent);
        driveF.SetPercent(xSign*percent);
        Sleep(tx);
        driveF.Stop();
        Sleep(ty-tx);
        driveL.Stop();
        driveR.Stop();
        */
        driveL.SetPercent(ySign*percent);
        driveR.SetPercent(-ySign*percent);
        Sleep(ty-tx);
        driveF.SetPercent(xSign*percent);
        Sleep(tx);
        driveL.Stop();
        driveR.Stop();
        driveF.Stop();

    }else{
        /*driveL.SetPercent(ySign*percent);
        driveR.SetPercent(-ySign*percent);
        driveF.SetPercent(xSign*percent);
        Sleep(ty);
        driveL.Stop();
        driveR.Stop();
        Sleep(tx-ty);
        driveF.Stop();
        */
        driveF.SetPercent(xSign*percent);
        Sleep(tx-ty);
        driveL.SetPercent(ySign*percent);
        driveR.SetPercent(-ySign*percent);
        Sleep(tx);
        driveF.Stop();
        driveL.Stop();
        driveR.Stop();


    }
}

void recordTheDarnLocationsInRPS(){
    char* places[] = {"jukeBoxBlue", "jukeBoxRed", "sink","sinkRotate","burger","burgerRotate","ticket","ticketRotate","rampExitTop","endButton"};
    double X,Y,H;
    FEHFile *cheetos = SD.FOpen("LOC.txt","w");
    if(!cheetos){
        LCD.WriteLine("sa;lfjha;ovrna;oernbv;lkeanrv;ouear");
    }
    //get rps coords of different places
    for(int i=0;i<11;i++){
        LCD.Clear();
        LCD.WriteLine(places[i]);
        generateLocationValues(&X,&Y,&H);
        SD.FPrintf(cheetos,"%s\n%f,%f,%f\n",places[i],X,Y,H);
        Sleep(500);
    }
    SD.FClose(cheetos);
}

void burgerFlip(double inchX,double inchY,double wheelPercent){

    

    //lower arm
    frontArm.SetDegree(90);

    //move to the right
    moveByTime(inchX,0,wheelPercent);

    //move back    
    moveByTime(0,inchY,-wheelPercent);

    //move further to the right
    moveByTime(inchX/2.0,0,wheelPercent);

    //move forward
    moveByTime(0,inchY,wheelPercent);

    //move left
    moveByTime(1.5*inchX,0,-wheelPercent);

    //raise arm
    frontArm.SetDegree(0);
}

//temporary thing to just get through the progress check
//requires that percent is at least 20%
void rotateByTime(double degrees, double percent){
    double revolutions = abs(ROTATION_CORRECTION_FACTOR*degrees/360.0);
    double sec = revolutions / (SLOPE_ROT*abs(percent) + B_ROT);

    if(sec>0){
        if(degrees > 0){
            driveL.SetPercent(percent);
            driveR.SetPercent(percent);
            driveF.SetPercent(percent);
            Sleep(sec);
            driveF.Stop();
            driveL.Stop();
            driveR.Stop();
        }
        else{
            driveL.SetPercent(-percent);
            driveR.SetPercent(-percent);
            driveF.SetPercent(-percent);
            Sleep(sec);
            driveF.Stop();
            driveL.Stop();
            driveR.Stop();
        }   
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

void generateLocationValues(double* x, double* y, double* heading){
    float a,b;
    //wait until the user touches the screen and releases
    LCD.ClearBuffer();
    while(!LCD.Touch(&a,&b)){}
    while(LCD.Touch(&a,&b)){}
    *x = RPS.X();
    *y = RPS.Y();
    *heading = RPS.Heading();

    //if it didn't read correctly, try again
    while(*x < 0){
        *x = RPS.X();
    }
    while(*y < 0){
        *y = RPS.Y();
    }
    while(*heading < 0){
        *heading = RPS.Heading();
    }
}

void goToLocation2(double xNew,double yNew,double headingNew,double power,char* name){
    FEHFile *testLog = SD.FOpen(name,"w");
    Sleep(RPS_DELAY);

    //update position values
    if(RPS.X() >= -1.1)
        x = RPS.X();

    if(RPS.Y() >= -1.1)
        y = RPS.Y();
     
    if(RPS.Heading() >= -1.1)
        heading = RPS.Heading();


    while(x > -2 && x < 0){
        x = RPS.X();
    }
    while(y > -2 && y < 0){
        y = RPS.Y();
    }
    while(heading > -2 && heading < 0){
        heading = RPS.Heading();
    }
    SD.FPrintf(testLog,"Moving from (%f,%f,%f) to (%f,%f,%f).\n",x,y,heading,xNew,yNew,headingNew);

    Sleep(RPS_DELAY);
    
    if(RPS.Heading() >= -1.1)
        heading = RPS.Heading();

    while(RPS.Heading() > -2 && RPS.Heading() < 0){
        heading = RPS.Heading();
    }    

    


    

    SD.FPrintf(testLog,"rotating from %f to %f via a rotation of %f\n",heading,headingNew,headingNew-heading);
    rotateByTime(headingNew-heading,power);
    Sleep(RPS_DELAY);
    
    if(RPS.Heading() >= -1.1)
        heading = RPS.Heading();
    else
        heading = headingNew;
    while(RPS.Heading() > -2 && RPS.Heading() < 0){
        heading = RPS.Heading();
    }    
    
    double theta = heading*PI/180.0;
    double X = (xNew-x)*sin(theta)-(yNew-y)*cos(theta);
    double Y = (xNew-x)*cos(theta)+(yNew-y)*sin(theta);

    SD.FPrintf(testLog,"With the current heading of %f, the robot will move by (%f,%f)\n",heading,X,Y);
    SD.FPrintf(testLog,"Which would result in the current change of coordinates (%f,%f)\n",X*sin(theta)-Y*cos(theta),X*cos(theta)+Y*sin(theta));
    moveByTime(X,Y,power);
    
    Sleep(RPS_DELAY);

    if(RPS.X() >= -1.1)
        x = RPS.X();
    else
        x=xNew;

    if(RPS.Y() >= -1.1)
        y = RPS.Y();
    else
        y=yNew;

    if(RPS.Heading() >= -1.1)
        heading = RPS.Heading();
    else
        heading = headingNew;

    //update position values
    while(RPS.X() > -2 && RPS.X() < 0){
        x = RPS.X();
    }
    while(RPS.Y() > -2 && RPS.Y() < 0){
        y = RPS.Y();
    }
    while(RPS.Heading() > -2 && RPS.Heading() < 0){
        heading = RPS.Heading();
    }   
    SD.FPrintf(testLog,"Current position is (%f,%f,%f)\n",x,y,heading);

    SD.FClose(testLog);

}

void goToLocation(double xNew,double yNew,double headingNew,double power,char* name){
    
    FEHFile *testLog = SD.FOpen(name,"w");
    //update position values
    while(RPS.X() > -2 && RPS.X() < 0){
        x = RPS.X();
    }
    while(RPS.Y() > -2 && RPS.Y() < 0){
        y = RPS.Y();
    }
    while(RPS.Heading() > -2 && RPS.Heading() < 0){
        heading = RPS.Heading();
    }

    Sleep(RPS_DELAY);

    //calculate angle and distance needed to go from current RPS point to specified point
    double tempHeading = 180.0*atan2(yNew-y,xNew-x)/PI;
    if(tempHeading < 0){
        tempHeading+=360;
    }
    double dist = sqrt((xNew-x)*(xNew-x) + (yNew-y)*(yNew-y));

    SD.FPrintf(testLog,"Moving from (%f,%f,%f) to (%f,%f,%f).\n",x,y,heading,xNew,yNew,headingNew);
    SD.FPrintf(testLog,"This requires an angle of %f and distance of %f.\n",tempHeading,dist);

    //rotate so that the robot gets that heading
    
    SD.FPrintf(testLog,"rotating from %f to %f\n",heading,tempHeading);

    if(tempHeading>heading)
        rotateByTime(tempHeading - heading,-power);    
    else
        rotateByTime(heading - tempHeading,power);
    //move forward by the distance between the points
    SD.FPrintf(testLog,"moving forward %f inches\n",dist);
    moveByTime(0,dist,power);
    

    while(RPS.Heading() > -2 && RPS.Heading() < 0){
        heading = RPS.Heading();
    }
    Sleep(RPS_DELAY);
    //rotate so that the robot's heading matches the needed heading
        SD.FPrintf(testLog,"rotating from %f to %f\n",heading,headingNew);


    if(headingNew>heading)
        rotateByTime(headingNew - heading,-power);    
    else
        rotateByTime(heading - headingNew,power);
    //update position values
    if(RPS.X() >= -1.1)
        x = RPS.X();
    else
        x=xNew;

    if(RPS.Y() >= -1.1)
        y = RPS.Y();
    else
        y=yNew;
     
    if(RPS.Heading() >= -1.1)
        heading = RPS.Heading();
    else
        heading=headingNew;

    while(RPS.X() > -2 && RPS.X() < 0){
        x = RPS.X();
    }
    while(RPS.Y() > -2 && RPS.Y() < 0){
        y = RPS.Y();
    }
    while(RPS.Heading() > -2 && RPS.Heading() < 0){
        heading = RPS.Heading();
    }

    Sleep(RPS_DELAY);
    SD.FPrintf(testLog,"New position is (%f,%f,%f)\n",x,y,heading);
    SD.FClose(testLog);
}
