#include <Arduino.h>

float tension;
float pressure;

void setup() {
  Serial.begin(115200);  // Start the Serial communication
}

void loop() {
  // Simulate some data readings; replace these with actual sensor readings
  tension = 3.3;  // Example tension value
  pressure = 1.2; // Example pressure value

  // Send the data over serial
  Serial.print(tension);
  Serial.print(",");  // A comma to separate the data values
  Serial.println(pressure);

  delay(1000);  // Wait for a second before sending the next set of data
}
