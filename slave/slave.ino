#pragma GCC optimize ("3")

#include <Wire.h>

#define LEDPIN A1

#define PWM_A OCR0A
#define PWM_B OCR0B
#define PWM_C OCR1A

uint16_t freq_counter = 0;
uint16_t old_freq_counter = 0;

// How fast is the main loop running.
uint16_t loop_time = 0;

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
uint16_t motor_step = 0;

// Actual motor speed, -127 to 127
int16_t motor_speed = 0;

// Motor speed limits.
int8_t min_speed = -100;
int8_t max_speed = 100;
int8_t speed = 60;

// Motor power limit is 255, but in order to save energy and reduce heat, the limit should be avoided.
uint8_t min_power = 80;
uint8_t max_power = 240;

// Power values are linearly interpolated from speed values:
// power = min_power + ((max_power - min_power) / (max_speed - min_speed)) * (speed - min_speed)
// In order to avoid useless computation, the power interpolation factor "((max_power - min_power) / (max_speed - min_speed))" is precomputed:
float power_interpol_factor = ((((float) max_power) - ((float) min_power)) / (((float) max_speed) - ((float) min_speed)));

int increment = 1;

void setup() {
  pinMode (LEDPIN, OUTPUT);

  Serial.begin(9600);

  // Join i2c bus with address #4.
  Wire.begin(4);
  // Register for i2c requests.
  Wire.onReceive(on_i2c_request);

  // Setup brushless motor Controller.
  init_motors();
}

void read_serial() {
  if (Serial.available() > 1) {
    // Read command.
    int32_t command = Serial.read();

    // Read value.
    int32_t value = Serial.read();

    switch(command) {
      case 's':
        digitalWrite(8, value > 0x77 ? HIGH : LOW);
        break;
      case 'r':
        speed = value;
        break;
      default:
        break;    
    }
  }
}

void loop() {
  // Fetch any serial command.
  read_serial();

  // Run main loop every ~4ms.
  if ((freq_counter & 0x07f) == 0) {
    // Record when loop starts.
    old_freq_counter = freq_counter;

    // // Main logic: set speed between -100 and 100 in order to allow the motor to run
    // if (speed >= max_speed) {
    //   increment = -1;
    // } else if (speed <= min_speed) {
    //   increment = 1;
    // }
    // speed += increment;

    // Actually run the motor.
    run_motor();

    // Calculate loop time.
    if (freq_counter > old_freq_counter) {
      if (loop_time < (freq_counter - old_freq_counter)) {
        loop_time = freq_counter - old_freq_counter;
      }
    }
  }
}

void on_i2c_request(int length) {
  // Safety check.  
  if (length != 2) {
    return;
  }

  // Read command.  
  char command = Wire.read();

  // Read value.
  int8_t value = Wire.read();

  switch(command) {
    case 's':
      digitalWrite(8, value > 0x77 ? HIGH : LOW);
      break;
    case 'r':
      speed = value;
      break;
    default:
      break;    
  }
}

void run_motor() {
  int16_t clamped_speed = constrain(speed, min_speed, max_speed);
  motor_speed = clamped_speed;

  // Interpolate values given the speed and power limits:
  motor_power = min_power + power_interpol_factor * (abs(clamped_speed) - min_speed);

  // Run motor.
  move_motor((uint8_t) (motor_step >> 8), motor_power);
}

void init_motors() {
  // Motor pins.
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);

  // Enable pin.
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  digitalWrite(LEDPIN, LOW);

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
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5
  OCR1A = 0;  //D9

  // Switch off PWM Power.
  stop_motor();
}

// Switch off motor power.
void stop_motor() {
  move_motor(0, 0);
}

void move_motor(uint8_t pos_step, uint16_t power) {
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
  PWM_A = (uint8_t) pwm_a;
  PWM_B = (uint8_t) pwm_b;
  PWM_C = (uint8_t) pwm_c;
}

// Interrupt code should be as small as possible.
// This is called every 31.875us (510 clock cycles)
ISR(TIMER1_OVF_vect) {
  // 32 counts of freq_counter are ~1ms.
  freq_counter++;

  if ((freq_counter & 0x01) == 0) {
    motor_step += motor_speed;
  }
}
