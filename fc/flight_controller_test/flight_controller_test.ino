///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>

TwoWire HWire (2, I2C_FAST_MODE);

//Let's declare some variables so we can use them in the complete program.
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer
uint8_t disable_throttle, flip32;
uint8_t error;
uint32_t loop_timer;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;
int16_t loop_counter;
uint8_t data, start, warning;
int16_t acc_axis[4], gyro_axis[4], temperature;
int32_t gyro_axis_cal[4], acc_axis_cal[4];
int32_t cal_int;
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
uint8_t channel_select_counter;

uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.


void setup() {
  pinMode(4, INPUT_ANALOG);
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.
  //The following function connects PB3 and PB4 to the alternate output function.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                     //Connects PB3 and PB4 to output function.

  //On the Flip32 the LEDs are connected differently. A check is needed for controlling the LEDs.
  pinMode(PB3, INPUT);                                         //Set PB3 as input.
  pinMode(PB4, INPUT);                                         //Set PB4 as input.
  //if (digitalRead(PB3) && digitalRead(PB4))flip32 = 1;         //Input PB3 and PB4 are high on the Flip32
  //else flip32 = 0;
  flip32 = 0;

  pinMode(PB3, OUTPUT);                                         //Set PB3 as output.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output.

  green_led(LOW);                                               //Set output PB3 low.
  red_led(LOW);                                                 //Set output PB4 low.

  Serial.begin(57600);                                          //Set the serial output to 57600 kbps.
  delay(100);                                                    //Give the serial port some time to start to prevent data loss.
  timer_setup();                                                //Setup the timers for the receiver inputs and ESC's output.
  delay(50);                                                    //Give the timers some time to start.

  HWire.begin();                                                //Start the I2C as master

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  HWire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  HWire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  HWire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  HWire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  HWire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  HWire.endTransmission();                                      //End the transmission with the gyro.


  print_intro();                                                //Print the intro on the serial monitor.
}

void loop() {
  delay(10);

  if (Serial.available() > 0) {
    data = Serial.read();                                       //Read the incomming byte.
    delay(100);                                                 //Wait for any other bytes to come in.
    while (Serial.available() > 0)loop_counter = Serial.read(); //Empty the Serial buffer.
    disable_throttle = 1;                                       //Set the throttle to 1000us to disable the motors.
  }

  if (!disable_throttle) {                                      //If the throttle is not disabled.
    TIMER4_BASE->CCR1 = channel_3;                              //Set the throttle receiver input pulse to the ESC 1 output pulse.
    TIMER4_BASE->CCR2 = channel_3;                              //Set the throttle receiver input pulse to the ESC 2 output pulse.
    TIMER4_BASE->CCR3 = channel_3;                              //Set the throttle receiver input pulse to the ESC 3 output pulse.
    TIMER4_BASE->CCR4 = channel_3;                              //Set the throttle receiver input pulse to the ESC 4 output pulse.
  }
  else {                                                        //If the throttle is disabled
    TIMER4_BASE->CCR1 = 1000;                                   //Set the ESC 1 output to 1000us to disable the motor.
    TIMER4_BASE->CCR2 = 1000;                                   //Set the ESC 2 output to 1000us to disable the motor.
    TIMER4_BASE->CCR3 = 1000;                                   //Set the ESC 3 output to 1000us to disable the motor.
    TIMER4_BASE->CCR4 = 1000;                                   //Set the ESC 4 output to 1000us to disable the motor.
  }

  if (data == 'a') {
    Serial.println(F("Reading receiver input pulses."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    reading_receiver_signals();
  }

  if (data == 'b') {
    Serial.println(F("Starting the I2C scanner."));
    i2c_scanner();
  }

  if (data == 'c') {
    Serial.println(F("Reading raw gyro data."));
    Serial.println(F("You can exit by sending a q (quit)."));
    read_gyro_values();
  }

  if (data == 'd') {
    Serial.println(F("Reading the raw accelerometer data."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    read_gyro_values();
  }

  if (data == 'e') {
    Serial.println(F("Reading the IMU angles."));
    Serial.println(F("You can exit by sending a q (quit)."));
    check_imu_angles();
  }

  if (data == 'f') {
    Serial.println(F("Test the LEDs."));
    test_leds();
  }

  if (data == 'g') {
    Serial.println(F("Reading the battery voltage."));
    Serial.println(F("You can exit by sending a q (quit)."));
    check_battery_voltage();
  }


  if (data == '1') {
    Serial.println(F("Check motor 1 (front right, counter clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '2') {
    Serial.println(F("Check motor 2 (rear right, clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '3') {
    Serial.println(F("Check motor 3 (rear left, counter clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '4') {
    Serial.println(F("Check motor 4 (front lefft, clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '5') {
    Serial.println(F("Check motor all motors."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }
}

void gyro_signalen(void) {
  //Read the MPU-6050 data.
  HWire.beginTransmission(gyro_address);                       //Start communication with the gyro.
  HWire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  HWire.endTransmission();                                     //End the transmission.
  HWire.requestFrom(gyro_address, 14);                         //Request 14 bytes from the MPU 6050.

  acc_axis[1] = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the acc_x variable.
  acc_axis[2] = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the acc_y variable.
  acc_axis[3] = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the acc_z variable.
  temperature = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the temperature variable.
  gyro_axis[1] = HWire.read() << 8 | HWire.read();             //Read high and low part of the angular data.
  gyro_axis[2] = HWire.read() << 8 | HWire.read();             //Read high and low part of the angular data.
  gyro_axis[3] = HWire.read() << 8 | HWire.read();             //Read high and low part of the angular data.
  gyro_axis[2] *= -1;                                          //Invert gyro so that nose up gives positive value.
  gyro_axis[3] *= -1;                                          //Invert gyro so that nose right gives positive value.

  if (cal_int >= 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];                            //Subtact the manual gyro roll calibration value.
    gyro_axis[2] -= gyro_axis_cal[2];                            //Subtact the manual gyro pitch calibration value.
    gyro_axis[3] -= gyro_axis_cal[3];                            //Subtact the manual gyro yaw calibration value.
  }
}
  void red_led(int8_t level) {
    if (flip32)digitalWrite(PB4, !level);
    else digitalWrite(PB4, level);
  }
  void green_led(int8_t level) {
    if (flip32)digitalWrite(PB3, !level);
    else digitalWrite(PB3, level);
  }
