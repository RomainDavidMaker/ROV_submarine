#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include "DShotESC.h"

#define baud_rate 115200
#define send_rate_ms 50 //rate to send sensor values via serial   (default 50)
#define screen_refresh_rate_ms 200   //screen refresh rate

#define max_time_serial_lost_ms 500 //shut motors if no data receive after this delay

/* pin definition  */
#define current_pin 36
#define pressure_pin 39
#define temp_esc_pin 34
#define temp_ext_pin 35
#define temp_bat_pin 32

/*
For the motors :

M1  front    M2
    M5  M6
    M8  M7
M4           M3
*/

#define esc1_pin GPIO_NUM_4
#define esc2_pin GPIO_NUM_25
#define esc3_pin GPIO_NUM_26
#define esc4_pin GPIO_NUM_18
#define esc5_pin GPIO_NUM_33
#define esc6_pin GPIO_NUM_16
#define esc7_pin GPIO_NUM_17
#define esc8_pin GPIO_NUM_27

#define led1_pin 12    //light front
#define led2_pin 2  //light bottom
#define voltage_pin 13 
#define servo_pin 23
#define SCL_pin 22
#define SDA_pin 21
#define echo_pin 19   //sonar
#define trig_pin 14
#define fan_pin 5
#define leak_pin 15



#define SCREEN_ADDR 0x3C  //  OLED I2C address
#define IMU_ADDR    0x4A  //  BNO080 I2C address

#define SCREEN_WIDTH 128  //screen dimension
#define SCREEN_HEIGHT 64


#define ESC_MAX 250  //max command to the motor 0 to 999 (default 250)


/* MOTOR DIRECTION 

CCW          CW
    CCW  CW
    CW   CCW 
CW           CCW

*/

#define ESC1_REVERSE true 
#define ESC2_REVERSE true
#define ESC3_REVERSE false
#define ESC4_REVERSE true
#define ESC5_REVERSE true
#define ESC6_REVERSE true
#define ESC7_REVERSE false
#define ESC8_REVERSE true


#define FAN_PWM_CHANEL 15    // default (15)
#define FAN_DUTY_CYCLE 120  // 0 to 255 (default 120)


void setup_screen();   //start  the screen

void setup_IMU();   //start the IMU

void setup_esc();   //setup the esc for the DSHOT600 protocol

void write_pos_motors_to_esc(); //write the pos to the esc

void send_test_esc() ; //send a sinusoidal position to all the motors

void read_motor_position();  //if serial is available, read the motors, lights and servo position

void read_analog_input();   //read all the analog inputs of the sensors

void read_IMU();

void send_sensor_values();  //send to the serial the position

float read_ntc_temp(int ntc_pin);  //

void display_sensors();  //display sensor values to the screen

void display_motors();