void move_left(double speed) {
  #ifdef roomba
    create_drive_direct(speed, 0);
  #ifndef
    motor(LEFT_WHEEL, speed);
  #endif
}

void move_right(double speed) {
  #ifdef roomba
    create_drive_direct(0, speed);
  #ifndef
    motor(RIGHT_WHEEL, speed);
  #endif
}
