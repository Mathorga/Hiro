#include <Wire.h>

#define LEDPIN 13

int8_t min_speed = -100;
int8_t max_speed = 100;
int8_t speed = 60;
int increment = 1;

void setup() {
  pinMode (LEDPIN, OUTPUT);
  Wire.begin(); // join i2c bus (address optional for master)
  start_motor();
}

void loop() {
  // Set motor speed.
  set_speed(speed);

  if (speed == 0) {
    digitalWrite(LEDPIN, HIGH);
    stop_motor();

    delay(2000);

    digitalWrite(LEDPIN, LOW);
    start_motor();
  }

  if (speed >= max_speed) {
    increment = -1;
  } else if (speed <= min_speed) {
    increment = 1;
  }
  speed += increment;

  delay(10);
}

void send_command(uint8_t address, char command, int8_t value) {
  // Start transmitting to device #<address>.
  Wire.beginTransmission(address);
  // Send command.
  Wire.write(command);
  // Send value.
  Wire.write(value);
  // End transmission.
  Wire.endTransmission();  
}

void start_motor() {
  send_command(4, 's', 0x7F);
}

void stop_motor() {
  send_command(4, 's', 0x00);
}

void set_speed(int8_t value) {
  send_command(4, 'r', value);
}