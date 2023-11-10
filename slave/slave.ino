#pragma GCC optimize ("3")

#include <Wire.h>

// Left motor pins.
#define L_MOTOR_A 5
#define L_MOTOR_B 6
#define L_MOTOR_C 9
#define L_ENABLE 8

// Right motor pins.
#define R_MOTOR_A 10
#define R_MOTOR_B 11
#define R_MOTOR_C 3
#define R_ENABLE 16

#define L_PWM_A OCR0A
#define L_PWM_B OCR0B
#define L_PWM_C OCR1A

#define R_PWM_A OCR1B
#define R_PWM_B OCR2A
#define R_PWM_C OCR2B

// Max command length.
#define COMMAND_SIZE 4


// --------------------------- SCHEDULING ---------------------------
uint16_t freq_counter = 0;
uint16_t old_freq_counter = 0;

// How fast is the main loop running.
uint16_t loop_time = 0;


// --------------------------- MOTORS ---------------------------
enum Motor {
  L = 0,
  R = 1
};
uint8_t motor_pwm_a[] = {L_PWM_A, R_PWM_A};
uint8_t motor_pwm_b[] = {L_PWM_B, R_PWM_B};
uint8_t motor_pwm_c[] = {L_PWM_C, R_PWM_C};
uint8_t motor_enable[] = {L_ENABLE, R_ENABLE};

// Space Vector PWM lookup table
// using uint8_t overflow for stepping
int8_t pwm_sin[] = {0,    5,    10,   16,   21,   27,   32,   37,   43,   48,   53,   59,   64,   69,   74,   79,
                    84,   89,   94,   99,   104,  109,  111,  112,  114,  115,  116,  118,  119,  120,  121,  122,
                    123,  123,  124,  125,  125,  126,  126,  126,  127,  127,  127,  127,  127,  127,  126,  126,
                    126,  125,  125,  124,  123,  123,  122,  121,  120,  119,  118,  116,  115,  114,  112,  111,
                    110,  112,  113,  114,  116,  117,  118,  119,  120,  121,  122,  123,  124,  124,  125,  125,
                    126,  126,  126,  127,  127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  124,  123,
                    122,  121,  120,  119,  118,  117,  116,  114,  113,  112,  110,  106,  101,  97,   92,   87,
                    82,   77,   72,   66,   61,   56,   51,   45,   40,   35,   29,   24,   18,   13,   8,    2,
                    -2,   -8,   -13,  -18,  -24,  -29,  -35,  -40,  -45,  -51,  -56,  -61,  -66,  -72,  -77,  -82,
                    -87,  -92,  -97,  -101, -106, -110, -112, -113, -114, -116, -117, -118, -119, -120, -121, -122,
                    -123, -124, -124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -126, -126, -126,
                    -125, -125, -124, -124, -123, -122, -121, -120, -119, -118, -117, -116, -114, -113, -112, -110,
                    -111, -112, -114, -115, -116, -118, -119, -120, -121, -122, -123, -123, -124, -125, -125, -126,
                    -126, -126, -127, -127, -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -123, -123,
                    -122, -121, -120, -119, -118, -116, -115, -114, -112, -111, -109, -104, -99,  -94,  -89,  -84,
                    -79,  -74,  -69,  -64,  -59,  -53,  -48,  -43,  -37,  -32,  -27,  -21,  -16,  -10,  -5,   0};

// 0 to 255, 255 means 100% power.
uint16_t motor_power = 0;

//motor pole angle, 0->255 overflow to loop after >>8 shift
uint16_t motor_step[] = {0, 0};

// Actual motor speed, -127 to 127
int16_t motor_speed[] = {0, 0};

// Motor speed limits.
int8_t min_speed = -100;
int8_t max_speed = 100;
int8_t speed[] = {60, 60};

// Motor power limit is 255, but in order to save energy and reduce heat, the limit should be avoided.
uint8_t min_power = 80;
uint8_t max_power = 240;

// Power values are linearly interpolated from speed values:
// power = min_power + ((max_power - min_power) / (max_speed - min_speed)) * (speed - min_speed)
// In order to avoid useless computation, the power interpolation factor "((max_power - min_power) / (max_speed - min_speed))" is precomputed:
float power_interpol_factor = ((((float) max_power) - ((float) min_power)) / (((float) max_speed) - ((float) min_speed)));


// --------------------------- SERIAL ---------------------------
// Current command.
char command[COMMAND_SIZE];

// New command data flag, used to know whether there's new serial data to read or not.
boolean new_data = false;

// Command starting character.
char command_starter = '{';

// Command ending character.
char command_ender = '}';

void setup() {
  // Enable outputs.
  pinMode(L_MOTOR_A, OUTPUT);
  pinMode(L_MOTOR_B, OUTPUT);
  pinMode(L_MOTOR_C, OUTPUT);
  pinMode(L_ENABLE, OUTPUT);
  pinMode(R_MOTOR_A, OUTPUT);
  pinMode(R_MOTOR_B, OUTPUT);
  pinMode(R_MOTOR_C, OUTPUT);
  pinMode(R_ENABLE, OUTPUT);

  Serial.begin(9600);

  // Setup brushless motor Controller.
  init_motors();
}

void read_serial() {
    // Tells whether the character receiving is ongoing or not.
    static boolean receiving = false;

    // Current char index.
    static byte char_index = 0;

    // Received character.
    char read_char;

    // Only look for new data when available.
    while (Serial.available() > 0 && new_data == false) {
        read_char = Serial.read();

        if (receiving == true) {
            if (read_char != command_ender) {
                command[char_index] = read_char;
                char_index++;
                if (char_index >= COMMAND_SIZE) {
                    char_index = COMMAND_SIZE - 1;
                }
            }
            else {
                command[char_index] = '\0'; // terminate the string
                receiving = false;
                char_index = 0;
                new_data = true;
            }
        } else if (read_char == command_starter) {
            receiving = true;
        }
    }
}

void show_command() {
    if (new_data == true) {
        Serial.print("This just in ... ");
        Serial.println(command);
        new_data = false;
    }
}

void read_command() {
    // Reads any new command.
    read_serial();

    new_data = false;

    char command_key = command[0];
    char* command_values = command + 1;

    switch (command_key) {
      // Enable motors.
      case 'e': {
          switch (command_values[0]) {
            case 'l':
              enable_motor(L);
              break;
            case 'r':
              enable_motor(R);
              break;
            case 'a':
              enable_motors();
          }
        }
        break;

      // Disable motors.
      case 'd': {
          switch (command_values[0]) {
            case 'l':
              disable_motor(L);
              break;
            case 'r':
              disable_motor(R);
              break;
            case 'a':
              disable_motors();
          }
        }
        break;

      // Run left motor.
      case 'l': {
          speed[L] = command_values[0];
        }
        break;
      // Run right motor.
      case 'r': {
          speed[R] = command_values[0];
        }
        break;
      default:
        break;
    }
}

void loop() {
  // Fetch any serial command.
  read_command();

  // Run main loop every ~4ms.
  if ((freq_counter & 0x07f) == 0) {
    // Record when loop starts.
    old_freq_counter = freq_counter;

    // Actually run the motor.
    run_motor(L);
    run_motor(R);

    // Calculate loop time.
    if (freq_counter > old_freq_counter) {
      if (loop_time < (freq_counter - old_freq_counter)) {
        loop_time = freq_counter - old_freq_counter;
      }
    }
  }
}

void enable_motors() {
  enable_motor(L);
  enable_motor(R);
}

void disable_motors() {
  disable_motor(L);
  disable_motor(R);
}

void enable_motor(uint8_t motor) {
  digitalWrite(motor_enable[motor], HIGH);
}

void disable_motor(uint8_t motor) {
  digitalWrite(motor_enable[motor], LOW);
}

void run_motor(uint8_t motor) {
  int16_t clamped_speed = constrain(speed[motor], min_speed, max_speed);
  motor_speed[motor] = clamped_speed;

  // Interpolate values given the speed and power limits:
  motor_power = min_power + power_interpol_factor * (abs(clamped_speed) - min_speed);

  // Run motor.
  move_motor(motor, (uint8_t) (motor_step[motor] >> 8), motor_power);
}

void init_motors() {
  // Disable motors.
  digitalWrite(L_ENABLE, LOW);

  // Stop interrupts.
  cli();

  // Timer setup for 31.250KHZ phase correct PWM.
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  // Enable Timer 1 interrupt.
  TIMSK1 = 0;
  TIMSK1 |= _BV(TOIE1);

  // Disable arduino standard timer interrupt.
  TIMSK0 &= ~_BV(TOIE1);

  // Start interrupts.
  sei();

  // Turn off all PWM signals.
  motor_pwm_a[L] = 0;
  motor_pwm_b[L] = 0;
  motor_pwm_c[L] = 0;
  motor_pwm_a[R] = 0;
  motor_pwm_b[R] = 0;
  motor_pwm_c[R] = 0;

  // Switch off PWM Power.
  stop_motor();
}

// Switch off motor power.
// TODO This is only called in the above function, consider removing this, since there's the enable pin available.
void stop_motor() {
  move_motor(L, 0, 0);
  move_motor(R, 0, 0);
}

void move_motor(uint8_t motor, uint8_t pos_step, uint16_t power) {
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;

  // Lookup sine values from table with 120deg offsets.
  pwm_a = pwm_sin[(uint8_t) pos_step];
  pwm_b = pwm_sin[(uint8_t) (pos_step + 85)];
  pwm_c = pwm_sin[(uint8_t) (pos_step + 170)];

  // Scale motor power.
  pwm_a = power * pwm_a;
  pwm_a = pwm_a >> 8;
  pwm_a += 128;

  pwm_b = power * pwm_b;
  pwm_b = pwm_b >> 8;
  pwm_b += 128;

  pwm_c = power * pwm_c;
  pwm_c = pwm_c >> 8;
  pwm_c += 128;

  // Set motor pwm variables.
  motor_pwm_a[motor] = (uint8_t) pwm_a;
  motor_pwm_b[motor] = (uint8_t) pwm_b;
  motor_pwm_c[motor] = (uint8_t) pwm_c;
}

// Interrupt code should be as small as possible.
// This is called every 31.875us (510 clock cycles)
ISR(TIMER1_OVF_vect) {
  // 32 counts of freq_counter are ~1ms.
  freq_counter++;

  if ((freq_counter & 0x01) == 0) {
    motor_step[L] += motor_speed[L];
    motor_step[R] += motor_speed[R];
  }
}
