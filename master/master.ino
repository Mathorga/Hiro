#define LEDPIN 13

#define I2C 0
#define SERIAL 1

int protocol = SERIAL;

int8_t min_speed = -100;
int8_t max_speed = 100;
int8_t speed = 60;
int increment = 1;

void setup() {
  pinMode (LEDPIN, OUTPUT);

  Serial.begin(9600);
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

void start_motor() {
  // Enable left motor.
  Serial.write("{el}");

  // Enable right motor.
  Serial.write("{er}");
}

void stop_motor() {
  // Disable left motor.
  Serial.write("{dl}");

  // Disable right motor.
  Serial.write("{dr}");
}

void set_speed(int8_t value) {
  if (value != 0) {
    // Left motor command.
    char l_command[] = {'{', 'l', value, '}'};
  
    // Right motor command.
    char r_command[] = {'{', 'r', value, '}'};
  
    Serial.write(l_command);
    Serial.write(r_command);
  }
}
