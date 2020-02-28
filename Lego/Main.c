//
//  Main.c
//  Botball 2018-2019
//
//  Created by RZJHS Robotics.
//  Copyright (C) 2019 RZJHS Robotics. All rights reserved.
//
#include <kipr/botball.h>
#include <math.h>
typedef enum { false, true } bool;
double pos[] = {0,0,0};
#define PI 3.141592653589793

/*
 * Motor Ports: 0 - Pulling Chain, 1 - Chain, 2 - Claw Wrist, 3 - Claw
 * Analog Ports: 0 - Light sensor, 1 - Left Line sensor, 2 - Right Line sensor
 * Digital Ports: 0 - Touch sensor
 */

#define CHAIN_PULL 0
#define CHAIN 1
#define CLAW_WRIST 2
#define CLAW 3

#define LIGHT_SENSOR 0
#define L_LINE_SENSOR 0
#define R_LINE_SENSOR 1

#define TOUCH_SENSOR 0

//PID Constants

//The Proportion Constant
#define kP 1.5

//The Integral Constant
#define kI 0.2

//The Derivative Constant
#define kD 0.01

//Constant for detecting when something is on the white
#define kStDev 50.0

//Change this variable from false to true if it is the actual competition
//Enables the line sensor
#define comp false


//Set this variable to false if you are using the Roomba, set this variable to true if you are using the Lego robot
#define robot true

//If you are using the Lego robot these ports must also be set:
//The left wheel port
#define lWheel 0

//The right wheel port
#define rWheel 1

double roombaLeftSpeed = 0.0;
double roombaRightSpeed = 0.0;

double dabs(double x) {
    return x>=0?x:-x;
};

void move_at_power(double lSpeed, double rSpeed) {
    if(robot) {
        motor_power(lWheel,lSpeed);
        motor_power(rWheel,rSpeed);
    } else {
        roombaLeftSpeed = lSpeed;
        roombaRightSpeed = rSpeed;
        create_drive_direct(lSpeed, rSpeed);
    };
}

void stop_moving() {
    if(robot) {
        ao();
    } else {
        roombaLeftSpeed = 0.0;
        roombaRightSpeed = 0.0;
        create_stop();
    };
}

void stop_moving_left() {
    if(robot) {
        motor_power(lWheel,0);
    } else {
        roombaLeftSpeed = 0.0;
        create_drive_direct(0,roombaRightSpeed);
    };
}

void stop_moving_right() {
    if(robot) {
        motor_power(rWheel,0);
    } else {
        roombaRightSpeed = 0.0;
        create_drive_direct(roombaLeftSpeed,0);
    };
}


//Reminder: as dt goes down, precision increases
double PID_control(double Error,double pError,double Integral,float dt) {
    double p = kP*Error;
    double i = kI*Integral;
    double d = kD*(Error-pError)/dt;
    return p+i+d;
}


int whiteValueL = 0;
int blackValueL = 0;
int whiteValueR = 0;
int blackValueR = 0;
double stDevL = 0.0;
double stDevR = 0.0;
//TODO: Test this function
void go_to_line(float lSpeed, float rSpeed, float dt) {
    double mL = analog(L_LINE_SENSOR);
    whiteValueL = mL;
    double mR = analog(R_LINE_SENSOR);
    whiteValueR = mR;
    move_at_power(lSpeed,rSpeed);
    float t = dt;
    stDevL = 0.0;
    stDevR = 0.0;
    msleep(1000.0*dt);
    while(t <= 0.1) {
        double sqNumL = (analog(L_LINE_SENSOR)-whiteValueL);
        double sqNumR = (analog(R_LINE_SENSOR)-whiteValueR);
        stDevL = (stDevL*(t-dt)+dt*sqNumL*sqNumL)/t;
        stDevR = (stDevR*(t-dt)+dt*sqNumR*sqNumR)/t;
        msleep(1000.0*dt);
        t += dt;
    };
    stDevL = sqrt(stDevL);
    stDevR = sqrt(stDevR);
    while(dabs(analog(L_LINE_SENSOR)-mL)<=kStDev*stDevL&&dabs(analog(R_LINE_SENSOR)-mR)<=kStDev*stDevR) {
        msleep(1000.0*dt);
        t += dt;
    };
    blackValueL = analog(L_LINE_SENSOR);
    blackValueR = analog(R_LINE_SENSOR);
}

void go_to_line_perpendicular(float lSpeed, float rSpeed, float dt) {
    double mL = analog(L_LINE_SENSOR);
    whiteValueL = mL;
    double mR = analog(R_LINE_SENSOR);
    whiteValueR = mR;
    move_at_power(lSpeed,rSpeed);
    float t = dt;
    stDevL = 0.0;
    stDevR = 0.0;
    msleep(1000.0*dt);
    while(t <= 0.1) {
        double sqNumL = (analog(L_LINE_SENSOR)-whiteValueL);
        double sqNumR = (analog(R_LINE_SENSOR)-whiteValueR);
        stDevL = (stDevL*(t-dt)+dt*sqNumL*sqNumL)/t;
        stDevR = (stDevR*(t-dt)+dt*sqNumR*sqNumR)/t;
        msleep(1000.0*dt);
        t += dt;
    };
    stDevL = sqrt(stDevL);
    stDevR = sqrt(stDevR);
    bool right = true;
    bool left = true;
    while(right || left) {
        if(dabs(analog(L_LINE_SENSOR)-mL)>kStDev*stDevL) {
            stop_moving_left();
            left = false;
        };
        if(dabs(analog(R_LINE_SENSOR)-mR)>kStDev*stDevR) {
            stop_moving_right();
            right = false;
        };
        msleep(1000.0*dt);
        t += dt;
    };
    blackValueL = analog(L_LINE_SENSOR);
    blackValueR = analog(R_LINE_SENSOR);
}

void follow_line(float Speed, float dist, float dt) {
    double pError = 0.0;
    double Integral = 0.0;
    float t = 0.0;
    for(t = 0.0;t<=dist/Speed;t+=dt) {
        float lSense = analog(L_LINE_SENSOR);
        float rSense = analog(R_LINE_SENSOR);
        if(lSense > blackValueL) {
            blackValueL = lSense;
        };
        if(rSense > blackValueR) {
            blackValueR = rSense;
        };
        double error = (analog(L_LINE_SENSOR)-analog(R_LINE_SENSOR))/4095.0;
        Integral += error*dt;
        double control = PID_control(error,pError,Integral,dt);
        pError = error;
        move_at_power(Speed*(1.0-control),Speed*(1.0+control));
        msleep(1000.0*dt);
    };
    stop_moving();
}

void get_firefighter(){
  move_at_power(-50,50);
	msleep(1000);
    move_at_power(-100,-100);
	msleep(1000);
    move_at_power(-50,50);
    msleep(900);
    move_at_power(100,100);
    msleep(1500);
}


void code() {
    //  msleep(until other robot moves);

   printf("in code");
    move_at_power(100,-100);
    msleep(775); // turns right
    printf("supposed to turn");

    move_at_power(100,100);

    msleep(5000);
    go_to_line(100,100,.01);
    move_at_power(100,98);
    msleep(250);// moves to utility zone

    move_at_power(-100,100);
    msleep(1000);



    /*go_to_line(100,100,.01);
    move_at_power(-100,100);
    msleep(400);// back up to line and turn left
*/
    move_at_power(100,100);
    msleep(1000);
    go_to_line(100,95,.01);

    move_at_power(-100,-65);
    msleep(2000);//turn to line


    follow_line(100,700,.01);// picks up crates

    move_at_power(-50,50);
    msleep(1600);// turns left

    // go parallel with box
    move_at_power(100,97);
    msleep(3175);

    move_at_power(-50,50);
    msleep(2000);// turns left


    go_to_line(50,49,.01);
    move_at_power(100,97);
    msleep(2000); //drives into DRZ box



    //back up
    move_at_power(-100,-97);
    msleep(2100);

    // do a 180
    move_at_power(-100,100);
    msleep(1730);

    //get fire truck
    go_to_line(100,99,.01);
    move_at_power(100,99);
    msleep(250);

    //align to verticle line
    move_at_power(0,-100);
    msleep(1400);
    move_at_power(100,100);
    msleep(4170);

    //turn and push into box
    move_at_power(-100,100);
    msleep(540);
    move_at_power(100,100);
    msleep(1500);

    //back up
    move_at_power(-100,-100);
    msleep(2000);

    //180 to get poms
    move_at_power(-100,100);
    msleep(2350);

    move_at_power(100,88);
    msleep(3750);
    move_at_power(100,100);
    msleep(8430);// get poms


    move_at_power(100,-100);
    msleep(825);

move_at_power(88,100);
    msleep(4450);  // goto utility box
    /*go_to_line(50,50,.001);
    msleep(1000);
    go_to_line(50,50,.001); //go to utility box*/
    move_at_power(-100,-100);
    msleep(1500);

    move_at_power(100,-100);
    msleep(1000);


    go_to_line(91,100,.01);
    follow_line(100,300,.01);
    move_at_power(100,100);
    msleep(1500);
    go_to_line(100,92,.01); //go to fire zone

    move_at_power(0,-100);
    msleep(1200);
    move_at_power(100,100);
    msleep(3000); // go to medical zone




    //turn and push into box
    move_at_power(-100,100);
    msleep(650);
    move_at_power(100,100);
    msleep(2000);








/*
    move_at_power(-100,-100);
    msleep(1500); //back out of box

	move_at_power(-100,100);
    msleep(1200);

	move_at_power(100,100);
    msleep(3250);

    move_at_power(100,-100);
	msleep(800);

   move_at_power(100,100);
    msleep(1500);

   get_firefighter();
    get_firefighter();
    get_firefighter();
    get_firefighter();
    get_firefighter();
    */

}


int main() {
    if(!robot) {
        create_connect();
    };
    enable_servos();
    if(comp) {
        wait_for_light(LIGHT_SENSOR);
    };
    code();
    disable_servos();
    if(!robot) {
        create_disconnect();
    };
    return 0;
}
