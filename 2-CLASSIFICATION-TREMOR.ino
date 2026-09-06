#include <Wire.h>
#include <ESP32Servo.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

#define SERVO1_PIN 15
#define SERVO2_PIN 2

Servo servo1;
Servo servo2;

float gyroXBias = 0;
float gyroYBias = 0;

float servo1Angle = 90;
float servo2Angle = 90;

void readGyro(int16_t &gx, int16_t &gy, int16_t &gz)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6);

  if (Wire.available() >= 6)
  {
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

void calibrate()
{
  Serial.println("KEEP MPU STILL");

  long sumX = 0;
  long sumY = 0;

  for (int i = 0; i < 300; i++)
  {
    int16_t gx, gy, gz;

    readGyro(gx, gy, gz);

    sumX += gx;
    sumY += gy;

    delay(10);
  }

  gyroXBias = sumX / 300.0;
  gyroYBias = sumY / 300.0;

  Serial.println("CALIBRATION DONE");
}

void setup()
{
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  // Setup servos
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);

  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);

  // Center
  servo1.write(90);
  servo2.write(90);

  delay(1000);

  calibrate();
}

void loop()
{
  int16_t gx, gy, gz;

  readGyro(gx, gy, gz);

  float gyroX = (gx - gyroXBias) / 131.0;
  float gyroY = (gy - gyroYBias) / 131.0;


  // -----------------------------
  // REMOVE SMALL SENSOR NOISE
  // -----------------------------

  if (abs(gyroX) < 8)
    gyroX = 0;

  if (abs(gyroY) < 8)
    gyroY = 0;


  // -----------------------------
  // SERVO 1 = UP / DOWN
  // -----------------------------

  // Increased from 0.025 to 0.05
  servo1Angle += gyroX * 0.05;


  // -----------------------------
  // SERVO 2 = LEFT / RIGHT
  // -----------------------------

  servo2Angle -= gyroY * 0.05;


  // -----------------------------
  // LIMIT SERVO MOVEMENT
  // -----------------------------

  servo1Angle = constrain(servo1Angle, 50, 130);
  servo2Angle = constrain(servo2Angle, 50, 130);


  // -----------------------------
  // SIMPLE TREMOR DETECTION
  // -----------------------------

  // Only detects and reports tremor.
  // DOES NOT affect servo movement.

  if (abs(gyroX) > 100 || abs(gyroY) > 100)
  {
    Serial.println("TREMOR DETECTED");
  }


  // -----------------------------
  // SEND POSITION TO SERVOS
  // -----------------------------

  servo1.write((int)servo1Angle);
  servo2.write((int)servo2Angle);


  // -----------------------------
  // DEBUG
  // -----------------------------

  Serial.print("GX = ");
  Serial.print(gyroX);

  Serial.print("   GY = ");
  Serial.print(gyroY);

  Serial.print("   S1 = ");
  Serial.print(servo1Angle);

  Serial.print("   S2 = ");
  Serial.println(servo2Angle);

  delay(20);
}