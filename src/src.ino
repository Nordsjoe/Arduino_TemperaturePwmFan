
// PWM code stolen from : https://create.arduino.cc/projecthub/tylerpeppy/25-khz-4-pin-pwm-fan-control-with-arduino-uno-3005a1

const byte OC1A_PIN = 9;
const byte OC1B_PIN = 10;

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency (Frequency in HZ!) (Set currently to 25kHZ)
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);

#define MIN_FAN_SPEED 30  // Minimum allowed fan speed (% duty cycle)
#define MAX_FAN_SPEED 100 // Maxium fan speed (% duty cycle)

// Temperature limits. Between these limits fan speed increases lineraly with temperature
#define TEMP_MIN 25.0  // Below this temperature fan runs at MIN_FAN_SPEED (deg C)
#define TEMP_MAX 40.0  // Abobe this temperature fan runs at MAX_FAN_SPEED (deg C)

// States
#define STATE_MAX_SPEED 0
#define STATE_MIN_SPEED 1
#define STATE_LINEAR 2

// Linear constants
const float divisor = max((TEMP_MAX - TEMP_MIN), 1.0);  // Make sure we don't divide with 0 by mistake
const float k = (MAX_FAN_SPEED - MIN_FAN_SPEED) / divisor;
const float m = MIN_FAN_SPEED - k * TEMP_MIN;

// Temp sensor code
const int sensorPin = A0;
// Temp sensor code end

void setup() {

  // Temp sensor code start
  Serial.begin(9600); // Open a serial port
  // Temp sensor code end

  pinMode(OC1A_PIN, OUTPUT);

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;

}

void loop() {
  static int state = STATE_MAX_SPEED;
  static byte pwmDuty = 100;
  // Temp sensor code start
  int sensorVal = analogRead(sensorPin);
  Serial.print("Sensor value: ");
  Serial.print(sensorVal);

  // Convert ADC reading to voltage
  float voltage = (sensorVal / 1024.0) * 5.0;
  Serial.print(", Volts: ");
  Serial.print(voltage);

  // Convert to degC
  float temperature = (voltage - .5) * 100;

  // Do some error handling of the calculated temperature. 
  // If temperature is negative or temperature > 100 degC the sensor is probably faulty.
  if (temperature < 0.0 || temperature > 100.0) 
  {
    temperature = 100.0;
    Serial.print("Unexpected temperature detected. Sensor faulty?");
  }
  
  Serial.print(", degrees C: ");
  Serial.print(temperature);
  // Temp sensor code end

  if (temperature < TEMP_MIN)
  {
    state = STATE_MIN_SPEED;
  }
  else if (temperature > TEMP_MIN && temperature < TEMP_MAX)
  {
    state = STATE_LINEAR;
  }
  else
  {
    // Default state, max speed
    state = STATE_MAX_SPEED;
  }


  switch (state) {
    case STATE_MIN_SPEED:
      setPwmDuty(MIN_FAN_SPEED);
      break;
    case STATE_LINEAR:
      pwmDuty = calcPwmDuty(temperature);
      setPwmDuty(pwmDuty);
      Serial.print(", PWM duty: ");
      Serial.println(pwmDuty);
      break;
    default:
      // Max speed, temperature above TEMP_MAX or faulty
      setPwmDuty(MAX_FAN_SPEED);
      break;
  }

}

void setPwmDuty(byte duty) {
  OCR1A = (word) (duty * TCNT1_TOP) / 100;
}

byte calcPwmDuty(float temperature) {
  // Linear
  float pwmDuty_float = k * temperature + m;
  // Convert to byte
  byte pwmDuty_byte = (byte)(pwmDuty_float + 0.5);

  // Saurate calculated PWM duty. It should be no smaller than MIN_FAN_SPEED and not
  // greater than MAX_FAN_SPEED.
  pwmDuty_byte = max(pwmDuty_byte, MIN_FAN_SPEED);
  pwmDuty_byte = min(pwmDuty_byte, MAX_FAN_SPEED);
  
  return pwmDuty_byte;
}
