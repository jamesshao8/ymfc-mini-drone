void test_leds(void) {
  data = 0;
  if (Serial.available() > 0) {                                                         //If serial data is available
    data = Serial.read();                                                               //Read the incomming byte
    delay(100);                                                                         //Wait for any other bytes to come in
    while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer
  }
  Serial.println(F("The red LED is now ON for 3 seconds"));
  red_led(HIGH);                                                 //Set output PB4 high.
  delay(3000);
  Serial.println(F(""));
  Serial.println(F("The green LED is now ON for 3 seconds"));
  red_led(LOW);                                                  //Set output PB4 low.
  green_led(HIGH);                                               //Set output PB3 high.
  delay(3000);
  green_led(LOW);                                                //Set output PB3 low.
  print_intro();
}
