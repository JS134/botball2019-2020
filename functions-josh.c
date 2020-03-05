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
  #endif
  #ifndef roomba
  move(left_speed, right_speed);
  double aSpeed = left_speed + right_speed;
  double time = distance / aSpeed;
  msleep(1000*time);
  stop_moving();
  #endif
};

void go_to_line(double left_speed, double right_speed) {
  double mL = analog(L_LINE_SENSOR);
  whiteValueL = mL;
  double mR = analog(R_LINE_SENSOR);
  whiteValueR = mR;
  move_at_power(lSpeed,rSpeed);
  float t = dt;
  stDevL = 0.0;
  stDevR = 0.0;
  clock_t cClock = clock();
  while(t <= 0.1) {
    double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
    cClock = clock();
    double sqNumL = (analog(L_LINE_SENSOR)-whiteValueL);
    double sqNumR = (analog(R_LINE_SENSOR)-whiteValueR);
    stDevL = (stDevL*(t-dt)+dt*sqNumL*sqNumL)/t;
    stDevR = (stDevR*(t-dt)+dt*sqNumR*sqNumR)/t;
    t += dt;
  };
  stDevL = sqrt(stDevL);
  stDevR = sqrt(stDevR);
  while(dabs(analog(L_LINE_SENSOR)-mL)<=kStDev*stDevL&&dabs(analog(R_LINE_SENSOR)-mR)<=kStDev*stDevR) {
  };
  blackValueL = analog(L_LINE_SENSOR);
  blackValueR = analog(R_LINE_SENSOR);
};

void follow_line(double speed, double dist) {
  #ifdef roomba
  double pError = 0.0;
  double Integral = 0.0;
  clock_t cClock = clock();
  int initDist = get_create_distance();
  set_create_distance(0);
  for(;get_create_distance() * kMod< dist;) {
    double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
    t += dt;
    cClock = clock();
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
  set_create_distance(initDist + get_create_distance());
  #endif
  #ifndef roomba
  double pError = 0.0;
  double Integral = 0.0;
  double t = 0.0;
  clock_t cClock = clock();
  for(;t<=dist/Speed;) {
    double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
    t += dt;
    cClock = clock();
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
  #endif
};