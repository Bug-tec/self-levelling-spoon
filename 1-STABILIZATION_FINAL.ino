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


// ========================================
// SERVO 2 TREMOR FILTER
// ========================================

float gyroYSlow = 0;
float gyroYTremor = 0;
float servo2Correction = 0;
unsigned long lastMovementTime = 0;

const unsigned long IDLE_TIME = 15000;


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


  // -----------------------------
  // SETUP SERVOS
  // -----------------------------

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);

  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);


  // Center
  servo1.write(90);
  servo2.write(90);

  delay(1000);

  calibrate();


  gyroYSlow = 0;
  servo2Correction = 0;
}


void loop()
{
  int16_t gx, gy, gz;

  readGyro(gx, gy, gz);


  // Convert gyro values
  float gyroX = (gx - gyroXBias) / 131.0;
  float gyroY = (gy - gyroYBias) / 131.0;

  // ========================================
// IDLE RESET
// ========================================

// Check if there is movement
if (abs(gyroX) > 8 || abs(gyroY) > 8)
{
  lastMovementTime = millis();
}

// If no movement for 15 seconds
if (millis() - lastMovementTime >= IDLE_TIME)
{
  servo1Angle = 90;
  servo2Correction = 0;
  servo2Angle = 90;

  servo1.write(90);
  servo2.write(90);
}


  // ========================================
  // REMOVE SMALL SENSOR NOISE
  // ========================================

  if (abs(gyroX) < 8)
    gyroX = 0;

  if (abs(gyroY) < 8)
    gyroY = 0;


  // ========================================
  // SERVO 1
  // UP / DOWN
  // ========================================

servo1Angle += gyroX * 0.0475;


  // Increased physical range
  servo1Angle = constrain(
  servo1Angle,
  60,
  115
);


  // ========================================
  // SERVO 2
  // LEFT / RIGHT
  // ========================================

  /*
     Separate slow intentional movement
     from fast tremor-like movement.
  */

  // 2% slower slow-motion tracking
  gyroYSlow = gyroYSlow * 0.951 + gyroY * 0.049;


  // Fast component
  gyroYTremor = gyroY - gyroYSlow;


  // Ignore tiny fast noise
  if (abs(gyroYTremor) < 10)
    gyroYTremor = 0;


  // ========================================
  // TREMOR COMPENSATION
  // ========================================

  // 5% faster than previous 0.06 gain
  servo2Correction -= gyroYTremor * 0.063;


  // Slowly return toward center
  servo2Correction *= 0.985;


  // Maximum correction
  servo2Correction = constrain(
    servo2Correction,
    -50,
    50
  );


  // Servo 2 position
  servo2Angle = 90 + servo2Correction;


  // Physical limit
  servo2Angle = constrain(
    servo2Angle,
    40,
    140
  );


  // ========================================
  // SEND TO SERVOS
  // ========================================

  servo1.write((int)servo1Angle);
  servo2.write((int)servo2Angle);


  // ========================================
  // DEBUG
  // ========================================

  Serial.print("GX = ");
  Serial.print(gyroX);

  Serial.print("   GY = ");
  Serial.print(gyroY);

  Serial.print("   GY tremor = ");
  Serial.print(gyroYTremor);

  Serial.print("   S1 = ");
  Serial.print(servo1Angle);

  Serial.print("   S2 = ");
  Serial.println(servo2Angle);


  delay(20);
}