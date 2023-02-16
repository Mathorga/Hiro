#include <Wire.h>

int8_t min_speed = -100;
int8_t max_speed = 100;
int8_t speed = 60;
int increment = 1;

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
}

void loop() {
  // Start transmitting to device #4.
  Wire.beginTransmission(4);
  // Send command.
  Wire.write('s');
  // Send value.
  Wire.write(speed);
  // End transmission.
  Wire.endTransmission();

  if (speed >= max_speed) {
    increment = -1;
  } else if (speed <= min_speed) {
    increment = 1;
  }
  speed += increment;

  delay(10);
}