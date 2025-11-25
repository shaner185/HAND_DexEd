#include <Encoder.h>

// === Motor A (X-axis, 250:1, Inverted) ===
const int PWM1A = 9;
const int PWM2A = 10;
const int ENCA1 = 2; //encoder pin A
const int ENCB1 = 3; //encoder pin B
Encoder motorEncoderA(ENCA1, ENCB1);



// === PD Gains ===
float KpA = 0.8;
float KdA = 0;


int maxPWM = 255;

// === Gear Ratios & Encoder Specs ===
const float COUNTS_PER_DEGREE_A = 1800.0 / 360.0;
const float DEGREE_BUFFER = 6.0;

// === Joystick Analog Inputs ===
const int joystickXPin = A0; // controls Motor A
const int joystickYPin = A1; // controls Motor B
float joystickMinDeg = -30.0;
float joystickMaxDeg =  30.0;

// === Jog Control Settings ===
float targetDegA = 0.0;
float targetDegB = 0.0;
const float stepGain = 0.05;  // degrees per update per unit deflection
const float deadzone = 30.0;  // joystick units (on scale 0–1023)

// Control timing
unsigned long lastControlUpdate = 0;
const unsigned long controlInterval = 10; // ms
long lastEncCountA = 0;

//Glove code
int potPins[] = {A0, A1, A2, A3, A4};
int pos = 0;
int closedPotValues[5];
int openPotValues[5];
int count = 0;
int switch1Value;

void setup() {
  Serial.begin(9600);
  pinMode(PWM1A, OUTPUT); pinMode(PWM2A, OUTPUT);

  motorEncoderA.write(0);

  DDRA = 0;
  PORTA = 255;
  while(count < 3) {
    switch1Value = (PINA & 0b1);
    if ((switch1Value == 0) && (count == 0)) {
      for (int i = 0; i < 5; i++) {
        openPotValues[i] = analogRead(potPins[i]);
        Serial.print("Open Pot ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(openPotValues[i]);
      }
      count++;
      delay(500);
    }
    else if ((switch1Value == 0) && (count == 1)) {
      
        closedPotValues[0] = analogRead(potPins[0]);
        Serial.print("Closed Pot ");
        Serial.print(0 + 1);
        Serial.print(": ");
        Serial.println(closedPotValues[0]);
      count++;
      delay(500);
    }
    else if ((switch1Value == 0) && (count == 2)) {
      for (int i = 1; i < 5; i++) {
        closedPotValues[i] = analogRead(potPins[i]);
        Serial.print("Closed Pot ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(closedPotValues[i]);
      }
      count++;
      delay(500);
    }
  }

}

void loop() {
  unsigned long now = millis();

  if (now - lastControlUpdate >= controlInterval) {
    lastControlUpdate = now;

    // === Read joystick ===
    int joystickRawX = analogRead(joystickXPin);  // 0–1023
    int joystickRawY = analogRead(joystickYPin);  // 0–1023
    float dx = joystickRawX - 512;
    float dy = joystickRawY - 512;

    // === Apply deadzone ===
    if (abs(dx) > deadzone) {
      // X-axis inverted: reverse direction
      targetDegA -= dx * stepGain * (controlInterval / 1000.0);
    }
    if (abs(dy) > deadzone) {
      targetDegB += dy * stepGain * (controlInterval / 1000.0);
    }

    // === Clamp target degrees === MAIN LINE THAT DICTATES HAND MOVEMENT
    int pointer = analogRead(A1);
    targetDegA = map(pointer, openPotValues[1], closedPotValues[1], 0, 600);

    long targetCountsA = targetDegA * COUNTS_PER_DEGREE_A;

    // === Motor A (X-axis) PD control ===
    long currentCountA = motorEncoderA.read();
    long errorA = targetCountsA - currentCountA;
    float errorDegA = errorA / COUNTS_PER_DEGREE_A;
    float velocityA = (currentCountA - lastEncCountA) / (controlInterval / 1000.0);
    lastEncCountA = currentCountA;

    if (abs(errorDegA) > DEGREE_BUFFER) {
      float controlA = KpA * errorA - KdA * velocityA;
      controlA = constrain(controlA, -maxPWM, maxPWM);
      setMotor(PWM1A, PWM2A, controlA);
    } else {
      setMotor(PWM1A, PWM2A, 0);
    }


    // Debug
    Serial.print("Target A: "); Serial.print(targetDegA);
    Serial.print("°, Actual A: "); Serial.print(currentCountA / COUNTS_PER_DEGREE_A);
  }
}

// === Motor driver ===
void setMotor(int pwm1, int pwm2, float pwmVal) {
  if (pwmVal > 0) {
    analogWrite(pwm1, pwmVal);
    analogWrite(pwm2, 0);
  } else if (pwmVal < 0) {
    analogWrite(pwm1, 0);
    analogWrite(pwm2, -pwmVal);
  } else {
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
  }
}

// === Helper ===
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
