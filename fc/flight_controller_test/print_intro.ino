void print_intro(void) {
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("          YMFC-32 quadcopter setup tool"));
  Serial.println(F("==================================================="));
  Serial.println(F("a = Read the receiver input pulses"));
  Serial.println(F("b = I2C scanner to detect any I2C sensors attached"));
  Serial.println(F("c = Read the raw gyro values"));
  Serial.println(F("d = Read the raw accelerometer values"));
  Serial.println(F("e = Check the IMU angles"));
  Serial.println(F("f = Test the LEDs"));
  Serial.println(F("g = Read the battery voltage input"));
  Serial.println(F("==================================================="));
  Serial.println(F("1 = Check motor 1 (front right, counter clockwise direction)"));
  Serial.println(F("2 = Check motor 2 (rear right, clockwise direction)"));
  Serial.println(F("3 = Check motor 3 (rear left, counter clockwise direction)"));
  Serial.println(F("4 = Check motor 4 (front left, clockwise direction)"));
  Serial.println(F("5 = Check all motors"));
  Serial.println(F("==================================================="));
  Serial.println(F("For support and questions: www.brokking.net"));
  Serial.println(F(""));
  if (!disable_throttle) {                                      //If the throttle is not disabled.
    Serial.println(F("==================================================="));
    Serial.println(F("     WARNING >>>THROTTLE IS ENABLED<<< WARNING"));
    Serial.println(F("==================================================="));
  }
}
