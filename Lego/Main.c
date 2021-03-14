#include <kipr/wombat.h>
#include <math.h>
#include <time.h>

#define PI 3.14159265359


//PID CONSTANTS
#define kP 4.1
#define kI 0.11
#define kD 0.000015548


//RANGE CONSTANT FOR GO TO LINE
#define kR 0.3


//CONSTANT MODIFYING DISTANCES ON ROOMBA
#define kMod 0.01


//LINE SENSOR VALUES
double blackValueL = 0.0;
double blackValueR = 0.0;
double whiteValueL = 0.0;
double whiteValueR = 0.0;
//#define comp
//#define roomba


//NON ROOMBA PORTS
#define LEFT_WHEEL 1
#define RIGHT_WHEEL 0


//LIGHT SENSOR PORT NUMBER
#define LIGHT_SENSOR 2


//LEFT LINE SENSOR PORT NUMBER
#define LEFT_LINE_SENSOR 1


//RIGHT LINE SENSOR PORT NUMBER
#define RIGHT_LINE_SENSOR 0


//LEFT CLAW PORT NUMBER
#define LEFT_CLAW 1


//RIGHT CLAW PORT NUMBER
#define RIGHT_CLAW 0


//LEFT CLAW OPEN VALUE
#define LEFT_CLAW_OPEN 0


//RIGHT CLAW OPEN VALUE
#define RIGHT_CLAW_OPEN 2047


//LEFT CLAW CLOSE VALUE
#define LEFT_CLAW_CLOSE 943


//RIGHT CLAW CLOSE VALUE
#define RIGHT_CLAW_CLOSE 1085

double dabs(double x) {
    return (x >= 0) ? x : -x;
};

void move_left(double speed) {
    #ifdef roomba
    create_drive_direct(speed, 0);
    #else
    motor(LEFT_WHEEL, speed);
    #endif
}

void move_right(double speed) {
    #ifdef roomba
    create_drive_direct(0, speed);
    #else
    motor(RIGHT_WHEEL, speed);
    #endif
}

void move(double left_speed, double right_speed) {
    #ifdef roomba
    create_drive_direct(left_speed, right_speed);
    #else
    move_left(left_speed);
    move_right(right_speed);
    #endif
}

void stop_moving_left() {
    move_left(0.0);
}

void stop_moving_right() {
    move_right(0.0);
}

void stop_moving() {
    move(0.0, 0.0);
}

#ifdef roomba
void turn_angle(double power, double angle) {
    double goal_angle = get_create_total_angle()+angle;
    double nPower = (angle > 0) ? power : -power;
    move(nPower, -nPower);
    if(angle > 0) {
        while(goal_angle > get_create_total_angle()) {
        };
    } else {
        while(goal_angle < get_create_total_angle()) {
        };
    };
    stop_moving();
}
#endif

double PID_control(double error, double previous_error, double integral, double dt) {
    double p = kP * error;
    double i = kI * integral;
    double d = kD * (error - previous_error) / dt;
    return p+i+d;
};

void move_distance(double left_speed, double right_speed, double distance) {
    #ifdef roomba
    clock_t cClock = clock();
    double cDist = 0;
    double error = 0;
    int oAngle = get_create_total_angle();
    set_create_total_angle(0); //change angle
    int pRoombaDist = get_create_distance();
    double integral = 0;
    while(cDist < distance) {
        double angle = get_create_total_angle() * pi / 180;
        double changeDistance = get_create_distance() - pRoombaDist;
        changeDistance *= kMod;
        pRoombaDist = get_create_distance();
        cDist += changeDistance * cos(angle);
        double pError = error;
        error += changeDistance * sin(angle);
        double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
        cClock = clock();
        integral += error * dt;
        double control = PID_control(error, pError, integral, dt);
        move(left_speed*(1.0+control),right_speed*(1.0-control));
    };
    set_create_total_angle(get_create_total_angle() + oAngle); //reset angle
    #else
    move(left_speed, right_speed);
    double aSpeed = sqrt(left_speed*left_speed + right_speed*right_speed);
    double time = distance / aSpeed;
    msleep(1000*time);
    stop_moving();
    #endif
};

void go_to_line(double left_speed, double right_speed) {
    move(left_speed,right_speed);
    double speed = sqrt(left_speed*left_speed+right_speed*right_speed);
    float t = 0.0;
    clock_t cClock = clock();

    //average calculation
    double whiteValueLTemp = 0.0;
    double whiteValueRTemp = 0.0;
    #define go_to_line_whiteValueCalcTime 0.03
    while(t <= go_to_line_whiteValueCalcTime) {
        double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
        cClock = clock();
        double mL = analog(LEFT_LINE_SENSOR);
        double mR = analog(RIGHT_LINE_SENSOR);
        whiteValueLTemp += mL * dt;
        whiteValueRTemp += mR * dt;
        t += dt;
    };
    whiteValueL = whiteValueLTemp/t;
    whiteValueR = whiteValueRTemp/t;

    //standard deviation calculation
    double stDevL = 0.0;
    double stDevR = 0.0;
    double stDev = 0.0;
    t = 0.0;
    cClock = clock();
    #define go_to_line_stDevCalcTime 0.03
    while(t <= go_to_line_stDevCalcTime) {
        double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
        cClock = clock();
        double sqNumL = (analog(LEFT_LINE_SENSOR)-whiteValueL);
        double sqNumR = (analog(RIGHT_LINE_SENSOR)-whiteValueR);
        double sqNum = sqNumL+sqNumR;
        stDevL += dt*sqNumL*sqNumL;
        stDevR += dt*sqNumR*sqNumR;
        stDev += dt*sqNum*sqNum;
        t += dt;
    };
    stDevL /= t;
    stDevR /= t;
    stDev /= t;

    //go to line
    t = 0.0;
    float integralErrorLeft = 0.0;//this should be the integral of error dt
    float integralErrorRight = 0.0;//this should be the integral of error dt
    unsigned int leftCont = 1;
    unsigned int rightCont = 1;
    cClock = clock();
    while(leftCont || rightCont) {
        double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
        cClock = clock();
        double valL = analog(LEFT_LINE_SENSOR);
        double valR = analog(RIGHT_LINE_SENSOR);
        t += dt;
        integralErrorLeft += (valL - whiteValueL)*dt*speed;
        integralErrorRight += (valR - whiteValueR)*dt*speed;
        integralErrorLeft *= exp(-dt*speed);
        integralErrorRight *= exp(-dt*speed);
        if(dabs(integralErrorLeft) > kR*stDevL) {
            leftCont = 0;
            blackValueL = analog(LEFT_LINE_SENSOR);
        }
        if(dabs(integralErrorRight) > kR*stDevR) {
            rightCont = 0;
            blackValueR = analog(RIGHT_LINE_SENSOR);
        }
    };
};

void follow_line(double speed, double dist) {
    #ifdef roomba
    #define follow_line_condition get_create_distance() * kMod< dist
    int initDist = get_create_distance();
    set_create_distance(0);
    #else
    #define follow_line_condition t<=dist/speed
    double t = 0.0;
    #endif
    clock_t cClock = clock();
    double pError = 0.0;
    double Integral = 0.0;
    while(follow_line_condition) {
        double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
        cClock = clock();
        #ifndef roomba
        t += dt;
        #endif
        float lSense = analog(LEFT_LINE_SENSOR);
        float rSense = analog(RIGHT_LINE_SENSOR);
        if(lSense > blackValueL) {
            blackValueL = lSense;
        };
        if(rSense > blackValueR) {
            blackValueR = rSense;
        };
        if(lSense < whiteValueL) {
            whiteValueL = lSense;
        };
        if(rSense < whiteValueR) {
            whiteValueR = rSense;
        };
        double error = (lSense-whiteValueL)/(blackValueL-whiteValueL);
        error -= (rSense-whiteValueR)/(blackValueL-whiteValueR);
        Integral += error*dt;
        double control = PID_control(error,pError,Integral,dt);
        pError = error;
        move(speed*(1.0-control),speed*(1.0+control));
    };
    stop_moving();
    #undef follow_line_condition
    #ifdef roomba
    set_create_distance(initDist + get_create_distance());
    #endif
};

void code() {
    //WRITE YOUR CODE HERE
    set_servo_position(LEFT_CLAW, LEFT_CLAW_OPEN);
    set_servo_position(RIGHT_CLAW, RIGHT_CLAW_OPEN);//opens claws to later grab pvc ring
    msleep(1000);//gives motors time to open
    
    go_to_line(50,50);//goes to black line to calibrate y axis
    move_distance(-50,-50,80);//moves backwards
    move_distance(50,0,145);//turns to face towards pvc ring
    move_distance(-50,-50,100);//moves backwards as a saftey net
    
    go_to_line(80,80);//goes to black line to calibrate position towards pvc ring
    printf("go_to_line 1 finished.\n");
    move_distance(100,100,190);//moves to pvc ring
    set_servo_position(LEFT_CLAW, LEFT_CLAW_CLOSE);
    set_servo_position(RIGHT_CLAW, RIGHT_CLAW_CLOSE);//closes claws to grab pvc ring
    msleep(1000);//gives motors time to close
    move_distance(-50,-50,100);
    go_to_line(-50,-50);//goes back to line to calibrate position backwards
    printf("go_to_line 2 finished.\n");
    move_distance(-100,-100,100);//goes backwards to turning location
    
    //move_distance code:
    move_distance(-100,0,120);
    msleep(500);
    move_distance(100,100,300);
    move_distance(0,100,100);
    msleep(500);
    move_distance(100,100,100);
    msleep(500);
    set_servo_position(LEFT_CLAW, LEFT_CLAW_OPEN);
    set_servo_position(RIGHT_CLAW, RIGHT_CLAW_OPEN);
    msleep(500);
    move_distance(-100,-100,120);
    msleep(500);
    set_servo_position(LEFT_CLAW, LEFT_CLAW_CLOSE);
    set_servo_position(RIGHT_CLAW, RIGHT_CLAW_CLOSE);
    msleep(500);
    
    
    //line follower code:
    //move_left(-100);//turns to face new black line in order to follow it
    //msleep(1200);
    //stop_moving_left();
    //printf("Part 1 finished.\n");
    //move_distance(100,100,270);//moves past line in front of desired line
    //go_to_line(30,30);//moves towards desired line in order to follow it
    //printf("go to line 3 finished \n");
    //move_distance(20,20,70);
    //follow_line(60,10);
    
};

int main() {
    #ifdef roomba
    create_connect();
    #endif
    enable_servos();
    #ifdef comp
    wait_for_light(LIGHT_SENSOR);
    shut_down_in(119);
    #endif
    code();
    disable_servos();
    #ifdef roomba
    create_disconnect();
    #endif
    return 0;
};
