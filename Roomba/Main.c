//
//  Main.c
//  Botball 2018-2019
//
//  Created by RZJHS Robotics.
//  Copyright B) 2019 RZJHS Robotics. All rights reserved.
//
#include <kipr/botball.h>
#include <math.h>
typedef enum { false, true } bool;
double pos[] = {0,0,0};
#define PI 3.141592653589793

#define CHAIN 0
#define RAISE_CHAIN 2
#define CLAW 1
#define CLAW_WRIST 3

#define L_LINE_SENSOR 0
#define R_LINE_SENSOR 1
#define LIGHT_SENSOR 2

#define TOUCH_SENSOR 0

//PID Constants

//The Proportion Constant
#define kP 2.3

//The Integral Constant
#define kI 1.75

//The Derivative Constant
#define kD 0.02

//Constant for detecting when something is on the white
#define kStDev 10.0

//Change this variable from false to true if it is the actual competition
//Enables the line sensor
#define comp false

//Set this variable to false if you are using the Roomba, set this variable to true if you are using the Lego robot
#define robot false

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

#define CHAIN_LIFT 2047
#define CHAIN_LOWER 924
#define CLAW_OPEN 1633
#define CLAW_CLOSE 750
#define CLAW_CLICK 600
#define WRIST_HORIZONTAL 650
#define WRIST_VERTICAL 1809
void lift_chain() {
    set_servo_position(RAISE_CHAIN,CHAIN_LIFT);
};

void lift_chain_slow() {
    int i = 0;
    for(i = 0;i<10;i++) {
    	set_servo_position(RAISE_CHAIN,(CHAIN_LIFT*i+CHAIN_LOWER*(10-i))/10.0);
        msleep(100);
    };
};

void lower_chain() {
    set_servo_position(RAISE_CHAIN,CHAIN_LOWER);
};

void lower_chain_slow() {
    int i = 0;
    for(i = 0;i<10;i++) {
    	set_servo_position(RAISE_CHAIN,(CHAIN_LOWER*i+CHAIN_LIFT*(10-i))/10.0);
        msleep(100);
    };
};

void wrist_horizontal() {
    set_servo_position(CLAW_WRIST,WRIST_HORIZONTAL);
};

void wrist_vertical() {
    set_servo_position(CLAW_WRIST,WRIST_VERTICAL);
};

void open_claw() {
    set_servo_position(CLAW,CLAW_OPEN);
};

void close_claw() {
    set_servo_position(CLAW,CLAW_CLOSE);
};

bool close_claw_until_button() {
    open_claw();
    double pos;
    for(pos = 0.0;pos<=1.0;pos+=0.01) {
        set_servo_position(CLAW,CLAW_OPEN+(CLAW_CLICK-CLAW_OPEN)*pos);
        if(digital(TOUCH_SENSOR)) {
            return true;
        };
        msleep(20);
    };
    msleep(100);
    return digital(TOUCH_SENSOR);
};

void spin_chain(double distance, double speed) {
    motor_power(CHAIN,speed);
    msleep(1000*dabs(distance/speed));
    motor_power(CHAIN,0);
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

double diff = 0.0;
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
        if(dabs(analog(L_LINE_SENSOR)-mL)>kStDev) {
            stop_moving_left();
            left = false;
        };
        if(dabs(analog(R_LINE_SENSOR)-mR)>kStDev) {
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
        double error = (analog(L_LINE_SENSOR)-analog(R_LINE_SENSOR)-diff)/4095.0;
        Integral += error*dt;
        double control = PID_control(error,pError,Integral,dt);
        pError = error;
        move_at_power(Speed*(1.0-control),Speed*(1.0+control));
        msleep(1000.0*dt);
    };
    stop_moving();
}

void initialize_position() {
    lift_chain();
    spin_chain(130,-100);
    wrist_horizontal();
    msleep(100);
    move_at_power(0,200);
    msleep(300);
    stop_moving();
    close_claw();
    lower_chain();
    motor_power(CHAIN,100);
    msleep(500);
    motor_power(CHAIN,0);
    spin_chain(210,-70);
};

void code() {
    initialize_position();
    move_at_power(200,200);
    msleep(2000);
    go_to_line(300,300,0.001);
    move_at_power(200,200);
    msleep(1400);
    stop_moving();
    move_at_power(0,-200);
    msleep(1700);
    stop_moving();
    move_at_power(-300,-300);
    msleep(1300);
    stop_moving();
    go_to_line(-300,-300,0.001);
    stop_moving();
    move_at_power(250,250);
    msleep(600);
    stop_moving();
    //add stuff
    lower_chain();
    open_claw();
    wrist_horizontal();
    spin_chain(240,50);
    msleep(1000);
    close_claw();
    msleep(500);
    spin_chain(50,-100);
    move_at_power(-100,-100);
    msleep(200);
    stop_moving();
    spin_chain(50,-100);
    move_at_power(-100,-100);
    msleep(200);
    stop_moving();
    spin_chain(50,-100);
    move_at_power(100,100);
    msleep(400);
    stop_moving();
    msleep(1000);
    move_at_power(-100,100);
    msleep(4000);
    stop_moving();
    move_at_power(-100,-100);
    msleep(400);
    stop_moving();
    spin_chain(80,100);
    open_claw();
    msleep(100);
    move_at_power(100,100);
    msleep(400);
    stop_moving();
    spin_chain(250,-100);
    spin_chain(60,-30);
    move_at_power(200,0);
    msleep(1700);
    stop_moving();
    //go_to_line(200,200,0.001);
    move_at_power(200,200);
    msleep(900);
    stop_moving();
    spin_chain(60,30);
    lift_chain_slow();
    lift_chain();
    spin_chain(100,-60);
    msleep(5000);//wait for other robot to move//should be 12000 in actual program
    move_at_power(-200,-200);
    msleep(1000);
    stop_moving();
    go_to_line(-200,-200,0.001);
    move_at_power(-200,0);
    msleep(1475);
    stop_moving();
    wrist_horizontal();
    move_at_power(150,150);
    msleep(1400);
    stop_moving();
    spin_chain(275,100);
    spin_chain(10,-100);
    move_at_power(-200,200);
    msleep(2100);
    stop_moving();
    move_at_power(-200,-200);
    msleep(2100);
    stop_moving();
    int tower = -1;
    //tower 1
    if(close_claw_until_button()) {
        close_claw();
        msleep(1000);
        printf("Tower 1 Active\n");
        spin_chain(70,-100);
        go_to_line(200,200,0.001);
        move_at_power(-200,200);
        msleep(1700);
        stop_moving();
        move_at_power(-200,-200);
        msleep(500);
        stop_moving();
        lower_chain_slow();
        lower_chain();
        msleep(500);
        open_claw();
        msleep(500);
        lift_chain_slow();
        lift_chain();
        move_at_power(200,200);
        msleep(500);
        stop_moving();
        spin_chain(100,100);
        move_at_power(200,-200);
        msleep(1700);
        stop_moving();
        move_at_power(200,200);
        msleep(1000);
        stop_moving();
        move_at_power(200,-50);
        msleep(1500);
        stop_moving();
    } else {
        tower = 0;
        open_claw();
        go_to_line(200,200,0.001);
        //turn
        move_at_power(200,200);
        msleep(1000);
        stop_moving();
        move_at_power(200,-50);
        msleep(1500);
        stop_moving();
    };
    follow_line(200,60,0.001); //finish this
    //Tower 2
    //turn
    move_at_power(-200,200);
    msleep(1100);
    stop_moving();
    //raise chain
    spin_chain(30,-100);
    //approach tower
    move_at_power(-200,-200);
    msleep(2100);
    stop_moving();
    if(close_claw_until_button()) {
        printf("Tower 2 Active\n");
        close_claw();
        msleep(1000);
        spin_chain(70,-100);
        go_to_line(200,200,0.001);
        move_at_power(-200,200);
        msleep(1900);
        stop_moving();
        move_at_power(-200,-200);
        msleep(500);
        stop_moving();
        lower_chain_slow();
        lower_chain();
        msleep(500);
        open_claw();
        msleep(500);
        lift_chain_slow();
        lift_chain();
        spin_chain(100,100);
        move_at_power(200,-200);
        msleep(1900);
        stop_moving();
        move_at_power(200,-50);
        msleep(1500);
        stop_moving();
    } else {
        tower = 1;
        open_claw();
        go_to_line(200,200,0.001);
        //turn
        move_at_power(200,200);
        msleep(1000);
        stop_moving();
        move_at_power(200,-50);
        msleep(1500);
        stop_moving();
    };
    follow_line(200,60,0.001); //finish this
    if(tower == -1) {
        //skip tower 3
        tower = 2;
    } else {
        //Tower 3
        //turn
        move_at_power(-200,200);
        msleep(1000);
        stop_moving();
        //lower chain
        spin_chain(10,100);
        //approach tower
        move_at_power(-200,-200);
        msleep(2100);
        stop_moving();
        if(close_claw_until_button()) {
            close_claw();
            msleep(1000);
            printf("Tower 3 Active\n");
            spin_chain(70,-100);
            go_to_line(200,200,0.001);
            move_at_power(200,-200);
            msleep(1700);
            stop_moving();
            move_at_power(-200,-200);
            msleep(500);
            stop_moving();
            lower_chain_slow();
            lower_chain();
            msleep(500);
            open_claw();
            msleep(500);
            lift_chain_slow();
            lift_chain();
            move_at_power(200,200);
            msleep(500);
            stop_moving();
            spin_chain(100,100);
            move_at_power(-200,200);
            msleep(1700);
            stop_moving();
            move_at_power(200,200);
            msleep(1000);
            sstop_moving();
            move_at_power(200,-50);
            msleep(1500);
            stop_moving();
        } else {
            tower = 2;
            open_claw();
            go_to_line(200,200,0.001);
            //turn
            move_at_power(200,200);
            msleep(1000);
            stop_moving();
            move_at_power(200,200);
            msleep(1000);
            stop_moving();
            move_at_power(200,-50);
            msleep(1500);
            stop_moving();
        };
    };
    //follow line
    follow_line(200,50,0.001); //finish this
    printf("Tower #%d is burning.\n",tower+1);
    move_at_power(-200,200);
    msleep(1500);
    stop_moving();
    move_at_power(200,200);
    msleep(1000);
    stop_moving();
    move_at_power(-200,200);
    msleep(1500);
    stop_moving();
    go_to_line(200,200,0.001);
    spin_chain(100,100);
    close_claw();
    spin_chain(100,-100);
    lift_chain();
    move_at_power(-200,200);
    msleep(1800);
    stop_moving();
    go_to_line(200,200,0.001);
    move_at_power(200,-50);
    msleep(1300);
    stop_moving();
    double followLineDist = 100.0*tower+100.0;
    follow_line(200,followLineDist,0.001);
    double chainDist = tower==1?60:30;
    spin_chain(chainDist,-100);
    move_at_power(200,-200);
    msleep(1100);
    stop_moving();
    move_at_power(-200,-200);
    msleep(1900);
    stop_moving();
    open_claw();
    go_to_line(200,200,0.001);
    move_at_power(200,-50);
    msleep(1300);
    stop_moving();
    follow_line(200,followLineDist,0.001);
    move_at_power(0,200);
    msleep(1900);
    stop_moving();
    spin_chain(100,100);
    //code to pick up gas valve
    close_claw();
    msleep(500);
    spin_chain(50,-100);
    move_at_power(-100,-100);
    msleep(200);
    stop_moving();
    spin_chain(50,-100);
    move_at_power(-100,-100);
    msleep(200);
    stop_moving();
    spin_chain(50,-100);
    move_at_power(100,100);
    msleep(400);
    stop_moving();
    spin_chain(100,-100);
    move_at_power(0,-200);
    msleep(1900);
    stop_moving();
    move_at_power(-200,200);
    msleep(1900);
    stop_moving();
    open_claw();
}

int main() {
    //diff = analog(L_LINE_SENSOR)-analog(R_LINE_SENSOR);
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