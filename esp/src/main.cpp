#include "main.h"

float motors_position[11] = {0.0};  //position 8 motor between -1 reverse, 0 stop ,1 forward, light front [0-1], light back, servo [-1-1]

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
BNO080 IMU;

float q0,q1,q2,q3 ; //quaternion real,i,j,k
int a =0;  //???
unsigned long t = 0;  //used to send data at constant rate
unsigned long t_screen = 0;  //used to refresh screen at constant rate
unsigned long t_last_command = 0;  //time the last command was receive

float current;  //mA
float voltage ; //voltage bat in V
float pressure; //atm
float temp_esc,temp_ext,temp_bat;//deg
float leak; //between 0 and 1 need to calibrate the trigger
float sonar_distance;  //cm




#define PEAKSPEED 100  //for esc
#define SINE_DURATION 10000.f //duration of the full cycle, in millis

DShotESC esc1;
DShotESC esc2;
DShotESC esc3;
DShotESC esc4;
DShotESC esc5;
DShotESC esc6;
DShotESC esc7;
DShotESC esc8;

void setup_esc()
{

	esc1.install(esc1_pin, RMT_CHANNEL_0);
  esc2.install(esc2_pin, RMT_CHANNEL_1);
  esc3.install(esc3_pin, RMT_CHANNEL_2);
  esc4.install(esc4_pin, RMT_CHANNEL_3);
  esc5.install(esc5_pin, RMT_CHANNEL_4);
  esc6.install(esc6_pin, RMT_CHANNEL_5);
  esc7.install(esc7_pin, RMT_CHANNEL_6);
  esc8.install(esc8_pin, RMT_CHANNEL_7);

	esc1.init();
  esc2.init();
  esc3.init();
  esc4.init();
  esc5.init();
  esc6.init();
  esc7.init();
  esc8.init();

	esc1.setReversed(ESC1_REVERSE);
  esc2.setReversed(ESC2_REVERSE);
  esc3.setReversed(ESC3_REVERSE);
  esc4.setReversed(ESC4_REVERSE);
  esc5.setReversed(ESC5_REVERSE);
  esc6.setReversed(ESC6_REVERSE);
  esc7.setReversed(ESC7_REVERSE);
  esc8.setReversed(ESC8_REVERSE);

	esc1.set3DMode(true);
  esc2.set3DMode(true);
  esc3.set3DMode(true);
  esc4.set3DMode(true);
  esc5.set3DMode(true);
  esc6.set3DMode(true);
  esc7.set3DMode(true);
  esc8.set3DMode(true);

	for (int i = 0; i < 5; i++)
	{
		esc1.beep(i);
    esc2.beep(i);
    esc3.beep(i);
    esc4.beep(i);
    esc5.beep(i);
    esc6.beep(i);
    esc7.beep(i);
    esc8.beep(i);
	}

}

void write_pos_motors_to_esc(){
  int16_t motors_position_int16[8] = {0};
  for(int i =0;i<8;i++){
    if (motors_position[i] > 1.0) motors_position[i] = 1.0; //check that the position are in -1 to 1
    if (motors_position[i] < -1.0) motors_position[i] = -1.0;
    motors_position_int16[i] = (int16_t)(motors_position[i]*ESC_MAX);
  }

  esc1.sendThrottle3D(motors_position_int16[0]);
  esc2.sendThrottle3D(motors_position_int16[1]);
  esc3.sendThrottle3D(motors_position_int16[2]);
  esc4.sendThrottle3D(motors_position_int16[3]);
  esc5.sendThrottle3D(motors_position_int16[4]);
  esc6.sendThrottle3D(motors_position_int16[5]);
  esc7.sendThrottle3D(motors_position_int16[6]);
  esc8.sendThrottle3D(motors_position_int16[7]);

}

void set_lights(){
  digitalWrite(led1_pin,int(motors_position[8]));
  digitalWrite(led2_pin,int(motors_position[9]));
}

void send_test_esc() {

	int16_t milliswrap = sin(millis()*2/SINE_DURATION*PI)*100;
	esc1.sendThrottle3D(milliswrap);
  esc2.sendThrottle3D(milliswrap);
  esc3.sendThrottle3D(milliswrap);
  esc4.sendThrottle3D(milliswrap);
  esc5.sendThrottle3D(milliswrap);
  esc6.sendThrottle3D(milliswrap);
  esc7.sendThrottle3D(milliswrap);
  esc8.sendThrottle3D(milliswrap);
	delay(1);
	
}

void setup() {
  Serial.begin(baud_rate);
  Wire.begin();
  
  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);


  ledcSetup(FAN_PWM_CHANEL, 150, 8); // configure FAN PWM at 150 Hz at 8 bits resolution
  
  ledcAttachPin(fan_pin, FAN_PWM_CHANEL);  // attach the channel to the GPIO
  ledcWrite(FAN_PWM_CHANEL, FAN_DUTY_CYCLE);  // set duty cycle of the fan

  setup_esc();
  setup_screen();
  setup_IMU();
  
}

void loop() {
  
  write_pos_motors_to_esc();
  read_analog_input();
  read_IMU();
  read_motor_position();
  set_lights();
  if (millis() - t > send_rate_ms) {
    send_sensor_values();
    t = millis();
  }

  if (millis() - t_screen > send_rate_ms) {
    if(t%6000 <3000) display_motors();
    else display_sensors();
    t_screen = millis();
  }

  if (millis() - t_last_command > max_time_serial_lost_ms) {  //if no data receive set motor position to 0
    for(int i =0;i<8;i++){
    motors_position[i] = 0;
  }
  }


}


void setup_screen(){

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR)) { 
    while (1)
      Serial.println(F("SSD1306 allocation failed. Freezing..."));
  }

}

void setup_IMU(){
  if (!IMU.begin())
  {
    while (1)
      Serial.println(F("BNO080 not detected at default I2C address. Freezing..."));
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  IMU.enableRotationVector(send_rate_ms); //Send data update every ...ms
}

void read_motor_position() {
    if (Serial.available() > 0) {
        t_last_command = millis(); //reset timer
        String inputString = Serial.readStringUntil('\n'); // Read the string until newline character
        int lastIndex = 0;
        int index = 0;
        for (int i = 0; i < 11; i++) {
            index = inputString.indexOf(';', lastIndex); // Find the next delimiter
            if (index == -1) {
                index = inputString.length(); // If no more delimiters, take till end of string
            }
            motors_position[i] = inputString.substring(lastIndex, index).toFloat(); // Extract and convert to float
            if (motors_position[i] > 1.0) motors_position[i] = 1.0;
            if (motors_position[i] < -1.0) motors_position[i] = -1.0;
            lastIndex = index + 1; // Move past the delimiter for next iteration
        }
    }
}


void read_analog_input(){
  current = analogRead( current_pin)*1;
   voltage = 11.+ (analogRead( voltage_pin)-1478.)/(1775.-1478.)*(13.-11.);  //afine relation calibration at 11v and 13V
   pressure= analogRead( pressure_pin)/212.;
   temp_esc= read_ntc_temp(temp_esc_pin);
   temp_bat= read_ntc_temp(temp_bat_pin);
   temp_ext= read_ntc_temp(temp_ext_pin);
   leak= analogRead( leak_pin)/4095.0;

}

void read_IMU(){
  if (
  IMU.dataAvailable() == true)
  {
    q0 = IMU.getQuatReal();
    q1 = IMU.getQuatI();
    q2 = IMU.getQuatJ();
    q3 = IMU.getQuatK();
  }
}

void send_sensor_values(){
  Serial.print(pressure);
  Serial.print(";");
  Serial.print(voltage);
  Serial.print(";");
  Serial.print(current);
  Serial.print(";");
  Serial.print(temp_ext);
  Serial.print(";");
  Serial.print(temp_bat);
  Serial.print(";");
  Serial.print(temp_esc);
  Serial.print(";");
  Serial.print(leak);
  Serial.print(";");
  Serial.print(q0);
  Serial.print(";");
  Serial.print(q1);
  Serial.print(";");
  Serial.print(q2);
  Serial.print(";");
  Serial.print(q3);
  Serial.print(";");
  Serial.print(sonar_distance);
  Serial.println();
}

float read_ntc_temp(int ntc_pin) { 
  #define U_0 3.3               //voltage when the adc reach 4095 (need to be calibrated)
  #define R_serie 100000.0       //ntc  with pullup resistor of value R_serie
  #define R0_ntc 100000.0     //nominal resistance of the NTC here 100K

  float v = analogRead(ntc_pin) * (U_0 / 4095.0);                       // Convert the analog value to voltage
  float r = R_serie / (U_0 / v - 1.0);                                  // Calculate the resistance of the NTC thermistor
  float t = 1.0 / ((1.0 / 298.15) + (1.0 / 3950.0) * log(r / R0_ntc));  // Calculate the temperature in Kelvin
  t -= 273.15;                                                          // Convert temperature to Celsius
  return t;
}



void display_sensors() {  //refresh display 

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); 

  display.setTextSize(3);               
  display.setCursor(0, 0); 
  display.print(voltage);
  display.println("V");
  display.setTextSize(2); 
  display.print(current);
  display.println("mA"); 
  display.print(max(temp_esc,temp_bat));
  display.println("oC"); 
  display.print(leak);
  display.println(" leak"); 

  display.display();
}



void display_motors() {
    const int bar_width = SCREEN_WIDTH / 8; // Width of each bar
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); 
    display.setTextSize(1); 
    display.println("1  2  3  4 5  6  7  8");  

    for (int i = 0; i < 8; i++) {
        int bar_height = int((motors_position[i]) * 26 ); // Draw the bar for each motor
        if(bar_height> 0 ) display.fillRect(i * bar_width, 37 -bar_height , bar_width - 1, bar_height , SSD1306_WHITE);
        if(bar_height< 0 ) display.fillRect(i * bar_width, 37 , bar_width - 1, -bar_height , SSD1306_WHITE);
    }
    display.display();
}

