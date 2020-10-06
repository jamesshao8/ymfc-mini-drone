void read_gyro_values(void) {
  cal_int = 0;                                                                        //If manual calibration is not used.

  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    delay(250);                                                                           //Print the receiver values on the screen every 250ms.
    if (Serial.available() > 0) {                                                         //If serial data is available.
      data = Serial.read();                                                               //Read the incomming byte.
      delay(100);                                                                         //Wait for any other bytes to come in.
      while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer.
    }
    if (data == 'c') {                                                                    //If the user requested a 'c'.
      if (cal_int != 2000) {
        gyro_axis_cal[1] = 0;                                                             //Reset calibration variables for next calibration.
        gyro_axis_cal[2] = 0;                                                             //Reset calibration variables for next calibration.
        gyro_axis_cal[3] = 0;                                                             //Reset calibration variables for next calibration.
        Serial.print("Calibrating the gyro");
        //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
          if (cal_int % 125 == 0) {                                                       //After every 125 readings.
            digitalWrite(PB3, !digitalRead(PB3));                                         //Change the led status to indicate calibration.
            Serial.print(".");                                                            //Print a dot to show something is still working.
          }
          gyro_signalen();                                                                //Read the gyro output.
          gyro_axis_cal[1] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
          gyro_axis_cal[2] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
          gyro_axis_cal[3] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.
          delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration
        }
        Serial.println(".");
        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
        gyro_axis_cal[1] /= 2000;                                                         //Divide the roll total by 2000.
        gyro_axis_cal[2] /= 2000;                                                         //Divide the pitch total by 2000.
        gyro_axis_cal[3] /= 2000;                                                         //Divide the yaw total by 2000.
        Serial.print("X calibration value:");
        Serial.println(gyro_axis_cal[1]);
        Serial.print("Y calibration value:");
        Serial.println(gyro_axis_cal[2]);
        Serial.print("Z calibration value:");
        Serial.println(gyro_axis_cal[3]);
      }
      gyro_signalen();                                                                    //Read the gyro output.
      Serial.print("Gyro_x = ");
      Serial.print(gyro_axis[1]);
      Serial.print(" Gyro_y = ");
      Serial.print(gyro_axis[2]);
      Serial.print(" Gyro_z = ");
      Serial.println(gyro_axis[3]);
    }
    else {                                                                                //If the user requested a 'd'.
      gyro_signalen();                                                                    //Read the accelerometer output.
      Serial.print("ACC_x = ");
      Serial.print(acc_axis[1]);
      Serial.print(" ACC_y = ");
      Serial.print(acc_axis[2]);
      Serial.print(" ACC_z = ");
      Serial.println(acc_axis[3]);
    }
  }
  print_intro();                                                                          //Print the intro to the serial monitor.

}


