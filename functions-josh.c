double PID_control(double error, double previous_error, double integral, double dt) {
  double p = kP * error;
  double i = kI * integral;
  double d = kD * (error - previous_error) / dt;
};

void move_distance(double speed, double distance) {
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
    pRoombaDist = get_create_distance()
    distance += changeDistance * cos(angle);
    double pError = error;
    error += changeDistance * sin(angle);
    double dt = ((double)(clock() - cClock)) / CLOCKS_PER_SEC;
    cClock = clock();
    integral += error * dt;
    double control = PID_control(error, pError, integral, dt);
    move(speed*(1.0-control),speed*(1.0+control));
    msleep(1000.0*dt);
  };
  set_create_total_angle(get_create_total_angle() + oAngle); //reset angle
  #endif
  #ifndef roomba
  #endif
};

void go_to_line(double left_speed, double right_speed) {
};

void align_line() {
};