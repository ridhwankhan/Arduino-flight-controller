#include <Wire.h>
#include <EEPROM.h>

float pid_p_gain_roll = 0.08; // Further lowered
float pid_i_gain_roll = 0.0003;
float pid_d_gain_roll = 1.2;
int pid_max_roll = 400;

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

float pid_p_gain_yaw = 0.6;
float pid_i_gain_yaw = 0.001;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;

boolean auto_level = true;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, gyro_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

ISR(PCINT0_vect) {
  current_time = micros();
  if(PINB & B00000001) {
    if(last_channel_1 == 0) {
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  } else if(last_channel_1 == 1) {
    last_channel_1 = 0;
    receiver_input[1] = current_time - timer_1;
    receiver_input_channel_1 = convert_receiver_channel(1);
  }
  if(PINB & B00000010) {
    if(last_channel_2 == 0) {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  } else if(last_channel_2 == 1) {
    last_channel_2 = 0;
    receiver_input[2] = current_time - timer_2;
    receiver_input_channel_2 = convert_receiver_channel(2);
  }
  if(PINB & B00000100) {
    if(last_channel_3 == 0) {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  } else if(last_channel_3 == 1) {
    last_channel_3 = 0;
    receiver_input[3] = current_time - timer_3;
    receiver_input_channel_3 = convert_receiver_channel(3);
  }
  if(PINB & B00001000) {
    if(last_channel_4 == 0) {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  } else if(last_channel_4 == 1) {
    last_channel_4 = 0;
    receiver_input[4] = current_time - timer_4;
    receiver_input_channel_4 = convert_receiver_channel(4);
  }
}

void gyro_signalen() {
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B);
  byte error = Wire.endTransmission();
  if(error != 0) {
    Serial.print("I2C Error: "); Serial.println(error);
    acc_x = acc_y = acc_z = 0;
    gyro_axis[1] = gyro_axis[2] = gyro_axis[3] = 0;
    return;
  }
  Wire.requestFrom(gyro_address, 14);
  unsigned long timeout = micros() + 2000;
  while(Wire.available() < 14 && micros() < timeout);
  if(Wire.available() >= 14 && eeprom_data[31] == 4) {
    acc_axis[1] = Wire.read()<<8|Wire.read();
    acc_axis[2] = Wire.read()<<8|Wire.read();
    acc_axis[3] = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyro_axis[1] = Wire.read()<<8|Wire.read();
    gyro_axis[2] = Wire.read()<<8|Wire.read();
    gyro_axis[3] = Wire.read()<<8|Wire.read();
  } else {
    Serial.print("No MPU data, Wire.available(): "); Serial.print(Wire.available());
    Serial.print(", eeprom_data[31]: "); Serial.println(eeprom_data[31]);
    acc_x = acc_y = acc_z = 0;
    gyro_axis[1] = gyro_axis[2] = gyro_axis[3] = 0;
    return;
  }
  if(cal_int == 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];
  }
  // Reassign axes to correct AccZ ~16384 when level
  acc_x = acc_axis[3]; // Map Z to X
  acc_y = acc_axis[1]; // Map X to Y
  acc_z = -acc_axis[2]; // Map Y to Z, invert for correct gravity
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];
  if(eeprom_data[28] & 0b10000000) gyro_roll *= -1;
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
  if(eeprom_data[29] & 0b10000000) gyro_pitch *= -1;
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];
  if(eeprom_data[30] & 0b10000000) gyro_yaw *= -1;
}

void calculate_pid() {
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;
  pid_last_roll_d_error = pid_error_temp;

  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;
  pid_last_pitch_d_error = pid_error_temp;

  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;
  pid_last_yaw_d_error = pid_error_temp;
}

int convert_receiver_channel(byte function) {
  byte channel, reverse;
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;
  if(eeprom_data[function + 23] & 0b10000000) reverse = 1;
  else reverse = 0;

  actual = receiver_input[channel];
  if(actual < 500 || actual > 2500) return 1500; // Default to center if invalid
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];

  if(function == 3 && actual <= low) return 1000;

  if(actual < center) {
    if(actual < low) actual = low;
    difference = ((long)(center - actual) * (long)500) / (center - low);
    if(reverse == 1) return 1500 + difference;
    else return 1500 - difference;
  } else if(actual > center) {
    if(actual > high) actual = high;
    difference = ((long)(actual - center) * (long)500) / (high - center);
    if(reverse == 1) return 1500 - difference;
    else return 1500 + difference;
  } else return 1500;
}

void set_gyro_registers() {
  Serial.println("Setting gyro registers...");
  if(eeprom_data[31] == 4) {
    Wire.beginTransmission(gyro_address);
    Wire.write(0x6B);
    Wire.write(0x00);
    byte error = Wire.endTransmission();
    Serial.print("Set 0x6B (PWR_MGMT_1): "); Serial.println(error == 0 ? "Success" : "Failed");
    delay(20);

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);
    Wire.write(0x08);
    error = Wire.endTransmission();
    Serial.print("Set 0x1B (GYRO_CONFIG): "); Serial.println(error == 0 ? "Success" : "Failed");
    delay(20);

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1C);
    Wire.write(0x10);
    error = Wire.endTransmission();
    Serial.print("Set 0x1C (ACCEL_CONFIG): "); Serial.println(error == 0 ? "Success" : "Failed");
    delay(20);

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1A);
    Wire.write(0x03);
    error = Wire.endTransmission();
    Serial.print("Set 0x1A (CONFIG): "); Serial.println(error == 0 ? "Success" : "Failed");
    delay(20);
  }
}

void setup() {
  Serial.begin(57600);
  Serial.println("Starting setup...");
  for(int i = 0; i <= 35; i++) eeprom_data[i] = EEPROM.read(i);
  Serial.print("EEPROM[31]: "); Serial.println(eeprom_data[31]);
  Serial.print("EEPROM[32]: "); Serial.println(eeprom_data[32], HEX);
  Serial.print("EEPROM[28]: "); Serial.println(eeprom_data[28], BIN);
  Serial.print("EEPROM[29]: "); Serial.println(eeprom_data[29], BIN);
  Serial.print("EEPROM[30]: "); Serial.println(eeprom_data[30], BIN);
  Serial.print("EEPROM[33]: "); Serial.println((char)eeprom_data[33]);
  Serial.print("EEPROM[34]: "); Serial.println((char)eeprom_data[34]);
  Serial.print("EEPROM[35]: "); Serial.println((char)eeprom_data[35]);

  Wire.begin();
  TWBR = 24; // 200kHz I2C
  gyro_address = eeprom_data[32];

  DDRD |= B11110000; // Pins 4-7 for ESCs
  DDRB |= B00110000; // Pin 12 for LED

  digitalWrite(12, HIGH);

  if(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B') {
    Serial.println("EEPROM signature invalid, continuing...");
  }

  set_gyro_registers();
  Serial.println("Gyro registers set");

  Serial.println("Initializing ESCs...");
  for (cal_int = 0; cal_int < 2000; cal_int++) {
    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delayMicroseconds(3000);
  }
  Serial.println("ESC initialization done");

  Serial.println("Calibrating gyro...");
  for (cal_int = 0; cal_int < 2000; cal_int++) {
    if(cal_int % 15 == 0) digitalWrite(12, !digitalRead(12));
    gyro_signalen();
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];
    gyro_axis_cal[3] += gyro_axis[3];
    delay(3);
  }
  gyro_axis_cal[1] /= 2000;
  gyro_axis_cal[2] /= 2000;
  gyro_axis_cal[3] /= 2000;
  Serial.print("Gyro cal: X="); Serial.print(gyro_axis_cal[1]);
  Serial.print(" Y="); Serial.print(gyro_axis_cal[2]);
  Serial.print(" Z="); Serial.println(gyro_axis_cal[3]);

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0); // Pin 8 (roll)
  PCMSK0 |= (1 << PCINT1); // Pin 9 (pitch)
  PCMSK0 |= (1 << PCINT2); // Pin 10 (throttle)
  PCMSK0 |= (1 << PCINT3); // Pin 11 (yaw)
  Serial.println("Interrupts enabled");

  battery_voltage = (analogRead(0) + 65) * 1.2317;
  Serial.print("Battery voltage: "); Serial.println(battery_voltage);

  loop_timer = micros();
  digitalWrite(12, LOW);
  Serial.println("Setup complete, entering loop...");
}

void loop() {
  gyro_roll_input = (gyro_roll_input * 0.98) + ((gyro_roll / 65.5) * 0.02);
  gyro_pitch_input = (gyro_pitch_input * 0.98) + ((gyro_pitch / 65.5) * 0.02);
  gyro_yaw_input = (gyro_yaw_input * 0.98) + ((gyro_yaw / 65.5) * 0.02);

  angle_pitch += gyro_pitch * 0.0000611;
  angle_roll += gyro_roll * 0.0000611;

  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);

  acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
  
  if(abs(acc_y) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_y/acc_total_vector) * 57.296;
  }
  if(abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x/acc_total_vector) * -57.296;
  }
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;

  if(!auto_level) {
    pitch_level_adjust = 0;
    roll_level_adjust = 0;
  }

  if(acc_total_vector < 1000 || acc_total_vector > 30000) {
    pid_output_roll = 0;
    pid_output_pitch = 0;
    pid_output_yaw = 0;
    pid_i_mem_roll = 0;
    pid_i_mem_pitch = 0;
    pid_i_mem_yaw = 0;
    Serial.println("Invalid MPU data, PID reset");
  }

  pid_roll_setpoint = 0;
  if(receiver_input_channel_1 > 1508) pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492) pid_roll_setpoint = receiver_input_channel_1 - 1492;
  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;

  pid_pitch_setpoint = 0;
  if(receiver_input_channel_2 > 1508) pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492) pid_pitch_setpoint = receiver_input_channel_2 - 1492;
  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;

  pid_yaw_setpoint = 0;
  if(receiver_input_channel_4 > 1508) pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
  else if(receiver_input_channel_4 < 1492) pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;

  calculate_pid();

  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  if(battery_voltage < 1050 && battery_voltage > 600) digitalWrite(12, HIGH);

  throttle = receiver_input_channel_3;
  if(throttle <= 1508 || throttle < 500) throttle = 1000; // Idle if invalid
  else throttle = map(throttle, 1508, 2000, 1000, 2000);
  if(throttle > 2000) throttle = 2000;

  esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
  esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
  esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
  esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

  if(battery_voltage < 1240 && battery_voltage > 800) {
    esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);
    esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);
    esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);
    esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);
  }

  if(esc_1 < 1000) esc_1 = 1000;
  if(esc_2 < 1000) esc_2 = 1000;
  if(esc_3 < 1000) esc_3 = 1000;
  if(esc_4 < 1000) esc_4 = 1000;

  if(esc_1 > 2000) esc_1 = 2000;
  if(esc_2 > 2000) esc_2 = 2000;
  if(esc_3 > 2000) esc_3 = 2000;
  if(esc_4 > 2000) esc_4 = 2000;

  Serial.print("Throttle: "); Serial.print(receiver_input_channel_3);
  Serial.print(" Yaw: "); Serial.print(receiver_input_channel_4);
  Serial.print(" Roll: "); Serial.print(receiver_input_channel_1);
  Serial.print(" Pitch: "); Serial.print(receiver_input_channel_2);
  Serial.print(" Roll Angle: "); Serial.print(angle_roll);
  Serial.print(" Pitch Angle: "); Serial.print(angle_pitch);
  Serial.print(" PID Roll: "); Serial.print(pid_output_roll);
  Serial.print(" PID Pitch: "); Serial.print(pid_output_pitch);
  Serial.print(" PID Yaw: "); Serial.println(pid_output_yaw);
  Serial.print("ESC1: "); Serial.print(esc_1);
  Serial.print(" ESC2: "); Serial.print(esc_2);
  Serial.print(" ESC3: "); Serial.print(esc_3);
  Serial.print(" ESC4: "); Serial.println(esc_4);
  Serial.print("AccX: "); Serial.print(acc_x);
  Serial.print(" AccY: "); Serial.print(acc_y);
  Serial.print(" AccZ: "); Serial.print(acc_z);
  Serial.print(" GyroX: "); Serial.print(gyro_axis[1]);
  Serial.print(" GyroY: "); Serial.print(gyro_axis[2]);
  Serial.print(" GyroZ: "); Serial.println(gyro_axis[3]);

  timer_channel_1 = esc_1 + loop_timer;
  timer_channel_2 = esc_2 + loop_timer;
  timer_channel_3 = esc_3 + loop_timer;
  timer_channel_4 = esc_4 + loop_timer;

  PORTD |= B11110000;
  while(PORTD >= 16) {
    esc_loop_timer = micros();
    if(timer_channel_1 <= esc_loop_timer) PORTD &= B11101111; // Pin 4
    if(timer_channel_2 <= esc_loop_timer) PORTD &= B11011111; // Pin 5
    if(timer_channel_3 <= esc_loop_timer) PORTD &= B10111111; // Pin 6
    if(timer_channel_4 <= esc_loop_timer) PORTD &= B01111111; // Pin 7
  }

  while(micros() - loop_timer < 4000); // 250Hz
  loop_timer = micros();
}