#pragma GCC optimize ("3")
#define LEDPIN A1

// Number of cells * lipo cell discharged voltage / ((5 / 1023) * (13.3 / 3.3))
#define dischargeVoltage  3 * 3.0 * 50.76

// Voltage to reset Bat_Discharged to false.
#define resetVoltage  3 * 3.7 * 50.76

#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;

// How fast is the main loop running.
uint16_t loop_time = 0;

uint16_t batteryVoltage = 0;
bool Bat_Discharged = false;      //true when (batteryVoltage < dischargeVoltage)

//IMU offset values
bool vertical = true;      //is the robot vertical enough to run

// Space Vector PWM lookup table
// using uint8_t overflow for stepping
int8_t pwmSinMotor[] = {0,    5,    10,   16,   21,   27,   32,   37,   43,   48,   53,   59,   64,   69,   74,   79,
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
uint16_t MotorPower = 120;

// Motor numbers.
#define L_Motor 0
#define R_Motor 1

//motor pole angle, 0->255 overflow to loop after >>8 shift
uint16_t R_MotorStep = 0;
uint16_t L_MotorStep = 0;

//rotation speed for turning
int8_t rot_Speed = 0;

// speed of motors, -127 to 127
int16_t R_Speed = 0;
int16_t L_Speed = 0;

// Motor speed limits.
int8_t min_speed = -100;
int8_t max_speed = 100;

// Motor power limit is 255, but in order to save energy and reduce heat, the limit should be avoided.
uint8_t min_power = 80;
uint8_t max_power = 240;

// Power values are linearly interpolated from speed values:
// power = min_power + ((max_power - min_power) / (max_speed - min_speed)) * (speed - min_speed)
// In order to avoid useless computation, the power interpolation factor "((max_power - min_power) / (max_speed - min_speed))" is precomputed:
float power_interpol_factor = ((((float) max_power) - ((float) min_power)) / (((float) max_speed) - ((float) min_speed)));

int8_t motor_speed = 60;

void setup() {
  pinMode (LEDPIN, OUTPUT);

  // Start Serial Port
  Serial.begin(230400);
  Serial.println("Bal_Code_v8_with_bluetooth_working_SVPWM" );
  Serial.println("place robot on front or back side withing 5sec. of power for gyroscope calibration");

  // Test battery voltage
  // testBattery();
  // Serial.print("Battery voltage = " );
  // Serial.println(batteryVoltage * 0.019698);


  // Setup brushless motor Controller.
  initMotors();

  // Empty RX buffer.
  while (Serial.available()) {
    Serial.read();
  }
}

int increment = 1;


void loop() {
  // Run main loop every ~4ms.
  if ((freqCounter & 0x07f) == 0) {
    // Record when loop starts.
    oldfreqCounter = freqCounter;

    // Test battery voltage every ~1s.
    // if ((freqCounter & 0x7FFF) == 0) {
    //   testBattery();
    // }

    // Run if on sent from smartphone and battery is charged.
    if (Bat_Discharged == false) {
      if (motor_speed >= max_speed) {
        increment = -1;
      } else if (motor_speed <= min_speed) {
        increment = 1;
      }

      motor_speed += increment;
      runMotors(motor_speed);
    } else {
      // Turn motors off if battery is discharged.
      stopMotors();
      MotorPower = 0;
    }


    // Calculate loop time.
    if (freqCounter > oldfreqCounter) {
      if (loop_time < (freqCounter - oldfreqCounter)) {
        loop_time = freqCounter - oldfreqCounter;
      }
    }
  }
}

void runMotors(int16_t motorSpeed) {
  if (vertical == true) {
    int16_t clamped_speed = constrain(motorSpeed, min_speed, max_speed);
    R_Speed = clamped_speed;
    L_Speed = clamped_speed;

    // Interpolate values given the speed and power limits:
    MotorPower = min_power + power_interpol_factor * (abs(clamped_speed) - min_speed);

    // Run motors.
    moveMotor(R_Motor, (uint8_t) (R_MotorStep >> 8), MotorPower);
    moveMotor(L_Motor, (uint8_t) (L_MotorStep >> 8), MotorPower);
  }

  else {
    // If not vertical turn everything off.
    stopMotors();
    MotorPower = 0;
  }
}

void initMotors() {
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  digitalWrite(LEDPIN, HIGH);

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
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5

  // Switch off PWM Power.
  stopMotors();
}

// Switch off motor power.
void stopMotors() {
  moveMotor(L_Motor, 0, 0);
  moveMotor(R_Motor, 0, 0);
}

void moveMotor(uint8_t motorNumber, uint8_t posStep, uint16_t power) {
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;

  // Lookup sine values from table with 120deg offsets.
  pwm_a = pwmSinMotor[(uint8_t) posStep];
  pwm_b = pwmSinMotor[(uint8_t) (posStep + 85)];
  pwm_c = pwmSinMotor[(uint8_t) (posStep + 170)];

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
  if (motorNumber == 0) {
    PWM_A_MOTOR0 = (uint8_t) pwm_a;
    PWM_B_MOTOR0 = (uint8_t) pwm_b;
    PWM_C_MOTOR0 = (uint8_t) pwm_c;
  }

  if (motorNumber == 1) {
    PWM_A_MOTOR1 = (uint8_t) pwm_a;
    PWM_B_MOTOR1 = (uint8_t) pwm_b;
    PWM_C_MOTOR1 = (uint8_t) pwm_c;
  }
}

void testBattery() {
  batteryVoltage = analogRead(A2);

  if (batteryVoltage < dischargeVoltage) {
    Bat_Discharged = true;

    // LED on when battery discharged.
    digitalWrite(LEDPIN, HIGH);
  } else {
    if (batteryVoltage > resetVoltage) {
      Bat_Discharged = false;
    }

    if (Bat_Discharged == false) {
      // Flash LED on/off every 2sec when battery charged.
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
    }
  }
}

// Interrupt code should be as small as possible.
// This is called every 31.875us (510 clock cycles)
ISR(TIMER1_OVF_vect) {
  // 32 counts of freqCounter are ~1ms.
  freqCounter++;

  if ((freqCounter & 0x01) == 0) {
    R_MotorStep += R_Speed;
    L_MotorStep += L_Speed;
  }
}
