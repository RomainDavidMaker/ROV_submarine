#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include "DShotESC.h"

#define baud_rate 115200
#define send_rate_ms 100

//pin definition
#define current_pin 36
#define pressure_pin 39
#define temp_esc_pin 34
#define temp_ext_pin 35
#define temp_bat_pin 32

#define esc1_pin GPIO_NUM_4
#define esc2_pin GPIO_NUM_25
#define esc3_pin GPIO_NUM_26
#define esc4_pin GPIO_NUM_18
#define esc5_pin GPIO_NUM_33
#define esc6_pin GPIO_NUM_16
#define esc7_pin GPIO_NUM_17
#define esc8_pin GPIO_NUM_27

#define led1_pin 2
#define led2_pin 12
#define voltage_pin 13
#define servo_pin 23
#define SCL_pin 22
#define SDA_pin 21
#define echo_pin 19
#define trig_pin 14
#define fan_pin 5
#define leak_pin 15

/*
M1  front    M2
    M5  M6
    M8  M7
M4           M3

*/

#define SCREEN_ADDR 0x3C  // Replace with your OLED I2C address
#define IMU_ADDR    0x4A  // Replace with your BNO080 I2C address

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


#define ESC_MAX 100  //max command to the motor 0 to 999

#define ESC1_REVERSE false  //change the motor direction
#define ESC2_REVERSE false
#define ESC3_REVERSE false
#define ESC4_REVERSE false
#define ESC5_REVERSE false
#define ESC6_REVERSE false
#define ESC7_REVERSE false
#define ESC8_REVERSE false


void setup_screen();
void setup_IMU();
void setup_esc();
void send_pos_motors_to_esc(); //write the pos to the esc
void send_test_esc() ; //send a sinusoidal position to all the motors
void read_motor_position();
void read_analog_input();
void read_IMU();
void send_sensor_values();
float read_ntc_temp(int ntc_pin);
void display_sensors();
void display_motors();