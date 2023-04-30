#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo_x;
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
float a = 0.8 ; //Complementary filter

void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  servo_x.attach(7);
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
   mpu.setThreshold(3);
}

void loop()
{
  timer = millis();
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();
  // Read normalized values
  Vector normGyro = mpu.readNormalizeGyro();

  int pitch_accel = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int roll_accel = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  // Calculate Pitch, Roll and Yaw
  int total_pitch = a*(pitch + normGyro.YAxis * timeStep) + (1-a)* pitch_accel;
  int total_roll = a*(roll + normGyro.XAxis * timeStep) + (1-a)*roll_accel;
  yaw = yaw + normGyro.ZAxis * timeStep;

  pitch=total_pitch;
  roll=total_roll;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);

  servo_x.write(pitch+90);
  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}
