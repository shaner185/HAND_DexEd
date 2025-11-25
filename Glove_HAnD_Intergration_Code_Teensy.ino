#include <Arduino.h>
#include <Encoder.h>

// === Motor A  ===
const int PWM1A = 0;
const int PWM2A = 1;
const int ENCA1 = 2;
const int ENCB1 = 3;
Encoder motorEncoderA(ENCA1, ENCB1);

// === Motor B  ===
const int PWM1B = 9;
const int PWM2B = 8;
const int ENCA2 = 11;
const int ENCB2 = 10;
Encoder motorEncoderB(ENCA2, ENCB2);

// === Motor C  ===
const int PWM1C = 29;
const int PWM2C = 28;
const int ENCA3 = 31;
const int ENCB3 = 30;
Encoder motorEncoderC(ENCA3, ENCB3);

// === Motor D ===
const int PWM1D = 37;
const int PWM2D = 36;
const int ENCA4 = 34;
const int ENCB4 = 35;
Encoder motorEncoderD(ENCA4, ENCB4);

// === Motor E  ===
const int PWM1E = 23;
const int PWM2E = 22;
const int ENCA5 = 21;
const int ENCB5 = 20;
Encoder motorEncoderE(ENCA5, ENCB5);



// === PD Gains ===
float KpA = 0.8;
float KdA = 0;
float KpB = 0.8;
float KdB = 0;
float KpC = 0.8;
float KdC = 0;
float KpD = 0.8;
float KdD = 0;
float KpE = 0.8;
float KdE = 0;


int maxPWM = 200;

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
float targetDegC = 0.0;
float targetDegD = 0.0;
float targetDegE = 0.0;
const float stepGain = 0.05;  // degrees per update per unit deflection
const float deadzone = 30.0;  // joystick units (on scale 0–1023)

// Control timing
unsigned long lastControlUpdate = 0;
const unsigned long controlInterval = 10; // ms
long lastEncCountA = 0;
long lastEncCountB = 0;
long lastEncCountC = 0;
long lastEncCountD = 0;
long lastEncCountE = 0;

//Glove code
int potPins[] = {A0, A1, A2, A3, A4};
int pos = 0;
int closedPotValues[5];
int openPotValues[5];
int count = 0;
int switch1Value;
const int button = 13;

void setup() {
  Serial.begin(9600);
  pinMode(PWM1A, OUTPUT); pinMode(PWM2A, OUTPUT);

  motorEncoderA.write(0);

  pinMode(button, INPUT_PULLDOWN);
  while(count < 3) {
    switch1Value = digitalRead(button);
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
    int thumb = analogRead(A0);
    int Pinky = analogRead(A4);
    int middle = analogRead(A2);
    int Ring = analogRead(A3);

    targetDegA = map(Pinky, openPotValues[4], closedPotValues[4], 0, 280);  
    targetDegB = map(Ring, openPotValues[3], closedPotValues[3], 0, 280);  
    targetDegC = map(pointer, openPotValues[1], closedPotValues[1], 0, 280);    
    targetDegD = map(thumb, openPotValues[0], closedPotValues[0], 0, 280);  
    targetDegE = map(middle, openPotValues[2], closedPotValues[2], 0, 280);

    long targetCountsA = targetDegA * COUNTS_PER_DEGREE_A;
    long targetCountsB = targetDegB * COUNTS_PER_DEGREE_A;
    long targetCountsC = targetDegC * COUNTS_PER_DEGREE_A;
    long targetCountsD = targetDegD * COUNTS_PER_DEGREE_A;
    long targetCountsE = targetDegE * COUNTS_PER_DEGREE_A;

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

   // === Motor B (Y-axis) PD control ===
    long currentCountB = motorEncoderB.read();
    long errorB = targetCountsB - currentCountB;
    float errorDegB = errorB / COUNTS_PER_DEGREE_A;
    float velocityB = (currentCountB - lastEncCountB) / (controlInterval / 1000.0);
    lastEncCountB = currentCountB;

    if (abs(errorDegB) > DEGREE_BUFFER) {
      float controlB = KpB * errorB - KdB * velocityB;
      controlB = constrain(controlB, -maxPWM, maxPWM);
      setMotor(PWM1B, PWM2B, controlB);
    } else {
      setMotor(PWM1B, PWM2B, 0);
    }

   // === Motor C (Y-axis) PD control ===
    long currentCountC = motorEncoderC.read();
    long errorC = targetCountsC - currentCountC;
    float errorDegC = errorC / COUNTS_PER_DEGREE_A;
    float velocityC = (currentCountC - lastEncCountC) / (controlInterval / 1000.0);
    lastEncCountC = currentCountC;

    if (abs(errorDegC) > DEGREE_BUFFER) {
      float controlC = KpC * errorC - KdC * velocityC;
      controlC = constrain(controlC, -maxPWM, maxPWM);
      setMotor(PWM1C, PWM2C, controlC);
    } else {
      setMotor(PWM1C, PWM2C, 0);
    }

   // === Motor D (Y-axis) PD control ===
    long currentCountD = motorEncoderD.read();
    long errorD = targetCountsD - currentCountD;
    float errorDegD = errorD / COUNTS_PER_DEGREE_A;
    float velocityD = (currentCountD - lastEncCountD) / (controlInterval / 1000.0);
    lastEncCountD = currentCountD;

    if (abs(errorDegD) > DEGREE_BUFFER) {
      float controlD = KpD * errorD - KdD * velocityD;
      controlD = constrain(controlD, -maxPWM, maxPWM);
      setMotor(PWM1D, PWM2D, controlD);
    } else {
      setMotor(PWM1D, PWM2D, 0);
    }

   // === Motor E (Y-axis) PD control ===
    long currentCountE = motorEncoderE.read();
    long errorE = targetCountsE - currentCountE;
    float errorDegE = errorE / COUNTS_PER_DEGREE_A;
    float velocityE = (currentCountE - lastEncCountE) / (controlInterval / 1000.0);
    lastEncCountE = currentCountE;

    if (abs(errorDegE) > DEGREE_BUFFER) {
      float controlE = KpE * errorE - KdE * velocityE;
      controlE = constrain(controlE, -maxPWM, maxPWM);
      setMotor(PWM1E, PWM2E, controlE);
    } else {
      setMotor(PWM1E, PWM2E, 0);
    }
    // Debug
    //Serial.print("Target A: "); Serial.print(targetDegA);
    //Serial.print("°, Actual A: "); Serial.print(currentCountA / COUNTS_PER_DEGREE_A);
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
