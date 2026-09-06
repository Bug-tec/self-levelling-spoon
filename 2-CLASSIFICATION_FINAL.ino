#include <Wire.h>

#define MPU6050_ADDR 0x68

#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43

// ========================================
// SETTINGS
// ========================================

const int SAMPLE_RATE = 100;

// 100 samples = 1 second
const int WINDOW_SIZE = 100;

// IMPORTANT:
// Gyro values in this program are degrees/sec,
// NOT raw MPU6050 values.
const float STABLE_THRESHOLD = 2.0;

// Tremor frequency range
const float TREMOR_MIN_FREQ = 3.0;
const float TREMOR_MAX_FREQ = 12.0;


// ========================================
// DATA
// ========================================

float gyroY[WINDOW_SIZE];
float gyroZ[WINDOW_SIZE];

float gyroYBias = 0;
float gyroZBias = 0;


// ========================================
// WRITE MPU6050 REGISTER
// ========================================

void writeRegister(byte reg, byte value)
{
  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
}


// ========================================
// READ GYROSCOPE
// ========================================

void readGyro(float &gy, float &gz)
{
  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(GYRO_XOUT_H);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 6);


  if (Wire.available() >= 6)
  {
    int16_t gxRaw =
      (Wire.read() << 8) | Wire.read();

    int16_t gyRaw =
      (Wire.read() << 8) | Wire.read();

    int16_t gzRaw =
      (Wire.read() << 8) | Wire.read();


    // ±250 deg/sec sensitivity
    gy = gyRaw / 131.0;
    gz = gzRaw / 131.0;
  }
}


// ========================================
// CALIBRATE
// ========================================

void calibrateGyro()
{
  Serial.println();
  Serial.println("==============================");
  Serial.println(" KEEP MPU6050 COMPLETELY STILL");
  Serial.println(" CALIBRATING...");
  Serial.println("==============================");

  delay(1000);


  float sumY = 0;
  float sumZ = 0;


  for (int i = 0; i < 300; i++)
  {
    float gy, gz;

    readGyro(gy, gz);

    sumY += gy;
    sumZ += gz;

    delay(5);
  }


  gyroYBias = sumY / 300.0;
  gyroZBias = sumZ / 300.0;


  Serial.println();
  Serial.println("CALIBRATION COMPLETE");
  Serial.println();
}


// ========================================
// REMOVE MEAN
// ========================================

void removeMean(float data[], int size)
{
  float mean = 0;


  for (int i = 0; i < size; i++)
  {
    mean += data[i];
  }


  mean /= size;


  for (int i = 0; i < size; i++)
  {
    data[i] -= mean;
  }
}


// ========================================
// CALCULATE RMS
// ========================================

float calculateRMS(float data[], int size)
{
  float sum = 0;


  for (int i = 0; i < size; i++)
  {
    sum += data[i] * data[i];
  }


  return sqrt(sum / size);
}


// ========================================
// FIND DOMINANT FREQUENCY
// ========================================

float findDominantFrequency(float data[], int size)
{
  float bestFrequency = 0;
  float bestMagnitude = 0;


  // Search 0.5 Hz → 12 Hz
  for (float frequency = 0.5;
       frequency <= 12.0;
       frequency += 0.5)
  {

    float real = 0;
    float imag = 0;


    for (int n = 0; n < size; n++)
    {

      float angle =
        2.0 * PI *
        frequency *
        n /
        SAMPLE_RATE;


      real +=
        data[n] * cos(angle);


      imag -=
        data[n] * sin(angle);
    }


    float magnitude =
      sqrt(
        real * real +
        imag * imag
      );


    if (magnitude > bestMagnitude)
    {
      bestMagnitude = magnitude;
      bestFrequency = frequency;
    }
  }


  return bestFrequency;
}


// ========================================
// SETUP
// ========================================

void setup()
{
  Serial.begin(115200);


  // ESP32 I2C
  Wire.begin(21, 22);


  // Wake MPU6050
  writeRegister(
    PWR_MGMT_1,
    0x00
  );


  // ±250 degrees/sec
  writeRegister(
    GYRO_CONFIG,
    0x00
  );


  Serial.println();
  Serial.println("==============================");
  Serial.println("   FAST TREMOR DETECTION");
  Serial.println("==============================");


  calibrateGyro();


  Serial.println("Detection started...");
  Serial.println();
}


// ========================================
// LOOP
// ========================================

void loop()
{
  unsigned long startTime = millis();


  // ========================================
  // COLLECT 1 SECOND OF DATA
  // ========================================

  for (int i = 0; i < WINDOW_SIZE; i++)
  {

    float gy;
    float gz;


    readGyro(gy, gz);


    // Remove gyro bias
    gyroY[i] =
      gy - gyroYBias;

    gyroZ[i] =
      gz - gyroZBias;


    // Maintain 100 Hz sample rate
    while (
      millis() - startTime <
      (unsigned long)((i + 1) * 10)
    )
    {
      delayMicroseconds(100);
    }
  }


  // ========================================
  // REMOVE DC / SLOW OFFSET
  // ========================================

  removeMean(
    gyroY,
    WINDOW_SIZE
  );

  removeMean(
    gyroZ,
    WINDOW_SIZE
  );


  // ========================================
  // MOVEMENT STRENGTH
  // ========================================

  float rmsY =
    calculateRMS(
      gyroY,
      WINDOW_SIZE
    );


  float rmsZ =
    calculateRMS(
      gyroZ,
      WINDOW_SIZE
    );


  // ========================================
  // FREQUENCY
  // ========================================

  float freqY =
    findDominantFrequency(
      gyroY,
      WINDOW_SIZE
    );


  float freqZ =
    findDominantFrequency(
      gyroZ,
      WINDOW_SIZE
    );


  // ========================================
  // CHOOSE STRONGER AXIS
  // ========================================

  float dominantRMS;
  float dominantFrequency;
  char dominantAxis;


  if (rmsY >= rmsZ)
  {
    dominantRMS = rmsY;
    dominantFrequency = freqY;
    dominantAxis = 'Y';
  }

  else
  {
    dominantRMS = rmsZ;
    dominantFrequency = freqZ;
    dominantAxis = 'Z';
  }


  // ========================================
  // CLASSIFICATION
  // ========================================

  String classification;


  // ----------------------------------------
  // VERY LITTLE MOVEMENT
  // ----------------------------------------

  if (dominantRMS < STABLE_THRESHOLD)
  {
    classification = "STABLE";
  }


  // ----------------------------------------
  // MOVEMENT IN TREMOR BAND
  // ----------------------------------------

  else if (
    dominantFrequency >= TREMOR_MIN_FREQ &&
    dominantFrequency <= TREMOR_MAX_FREQ
  )
  {
    classification = "TREMOR";
  }


  // ----------------------------------------
  // INTENTIONAL / SMOOTH MOVEMENT
  // ----------------------------------------

  else
  {
    classification = "SMOOTH";
  }


  // ========================================
  // SERIAL MONITOR
  // ========================================

  Serial.println();
  Serial.println("--------------------------------");


  Serial.print("Y RMS: ");

  Serial.print(
    rmsY,
    2
  );


  Serial.print(
    "   Y Frequency: "
  );

  Serial.print(
    freqY,
    1
  );

  Serial.println(" Hz");


  // ----------------------------------------

  Serial.print("Z RMS: ");

  Serial.print(
    rmsZ,
    2
  );


  Serial.print(
    "   Z Frequency: "
  );

  Serial.print(
    freqZ,
    1
  );

  Serial.println(" Hz");


  // ----------------------------------------

  Serial.print(
    "Dominant Axis: "
  );

  Serial.println(
    dominantAxis
  );


  Serial.print(
    "Dominant Frequency: "
  );

  Serial.print(
    dominantFrequency,
    1
  );

  Serial.println(" Hz");


  Serial.print(
    "Movement RMS: "
  );

  Serial.println(
    dominantRMS,
    2
  );


  // ========================================
  // COLOUR CLASSIFICATION
  // ========================================

  Serial.print("Classification: ");


  if (classification == "STABLE")
  {
    // GREEN
    Serial.print("\033[32m");

    Serial.println(
      ">>> STABLE <<<"
    );

    Serial.print("\033[0m");
  }


  else if (classification == "SMOOTH")
  {
    // YELLOW
    Serial.print("\033[33m");

    Serial.println(
      ">>> SMOOTH <<<"
    );

    Serial.print("\033[0m");
  }


  else
  {
    // RED
    Serial.print("\033[31m");

    Serial.println(
      ">>> TREMOR <<<"
    );

    Serial.print("\033[0m");
  }


  Serial.println("--------------------------------");
}