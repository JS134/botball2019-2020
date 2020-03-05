void move_left(double speed) {
  #ifdef roomba
    create_drive_direct(speed, 0);
  #endif
  #ifndef
    motor(LEFT_WHEEL, speed);
  #endif
}

void move_right(double speed) {
  #ifdef roomba
    create_drive_direct(0, speed);
  #endif
  #ifndef
    motor(RIGHT_WHEEL, speed);
  #endif
}

void move(double left_speed, double right_speed) {
  #ifdef roomba
    create_drive_direct(left_speed, right_speed);
  #endif
  #ifndef
    move_left(left_speed);
    move_right(right_speed);
  #endif
}

void stop_moving_left() {
  move_left(0);
}

void stop_moving_right() {
  move_right(0);
}

void stop_moving() {
  move(0, 0);
}
