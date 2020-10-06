void check_battery_voltage(void) {
  loop_counter = 0;                                                                       //Reset the loop counter.
  battery_voltage = analogRead(4);                                                        //Set battery voltage.
  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    delayMicroseconds(4000);                                                              //Wait for 4000us to simulate a 250Hz loop.
    if (Serial.available() > 0) {                                                         //If serial data is available.
      data = Serial.read();                                                               //Read the incomming byte.
      delay(100);                                                                         //Wait for any other bytes to come in.
      while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer.
    }
    loop_counter++;
    if (loop_counter == 250) {                                                            //Print the battery voltage every second.
      Serial.print("Voltage = ");                                                         //Print some preliminary information.
      Serial.print(battery_voltage / 112.81, 1);                                          //Print the avarage battery voltage to the serial monitor.
      Serial.println("V");                                                                //Print some trailing information.
      loop_counter = 0;                                                                   //Reset the loop counter.
    }
    //A complimentary filter is used to filter out the voltage spikes caused by the ESC's.
    battery_voltage = (battery_voltage * 0.99) + ((float)analogRead(4) * 0.01);
  }
  loop_counter = 0;                                                                       //Reset the loop counter.
  print_intro();                                                                          //Print the intro to the serial monitor.
}
