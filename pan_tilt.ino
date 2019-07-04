#include <Wire.h> //for i2c
#include <Servo.h> //Servo library
#include <math.h> //math library

const int MPUAddress = 0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,Tmp;

#define POWER 0x6B
#define ACCEL_XOUT_HIGH 0x3B

Servo motorRoll;  // roll motor
Servo motorPitch; // pitch motor

//Used for complementary filter calculations
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  

void set_last_read_angle_data(unsigned long time, float x, float y, float z) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
}

void setup(){
  
  //Servo setup
  motorRoll.attach(9); //blue servo
  motorPitch.attach(6); //dark blue servo

  //IMU setup
  Wire.begin();
  Wire.beginTransmission(MPUAddress);
  Wire.write(POWER);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop(){

  float GYRO_FACTOR = 131;  
  float RAD2DEGREE = 180/3.14159;
  
  Wire.beginTransmission(MPUAddress);
  Wire.write(ACCEL_XOUT_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(MPUAddress,14,true);
  
  AcX = Wire.read()<<8|Wire.read();      
  AcY = Wire.read()<<8|Wire.read();  
  AcZ = Wire.read()<<8|Wire.read();  
  Tmp = Wire.read()<<8|Wire.read();  
  GyX = (Wire.read()<<8|Wire.read())/GYRO_FACTOR;  
  GyY = (Wire.read()<<8|Wire.read())/GYRO_FACTOR;  
  GyZ = (Wire.read()<<8|Wire.read())/GYRO_FACTOR;  

  //debug printing all desired raw values
  /*
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(333);
  */
  
  //Calculations to get angular position of object with IMU:

  // Get current time
  unsigned long t_now = millis();
  
  //Accelerometer Y (-rho). turning around y axis results in vector on x axis
  float comp_accel_y = atan(-1.0*AcX/sqrt(pow(AcY,2) + pow(AcZ,2)))*RAD2DEGREE;
  //Serial.print("accel_y: "); Serial.print(comp_accel_y);
  
  //Accelerometer X (phi). turning around x axis results in vector on y axis
  float comp_accel_x = atan(AcY/sqrt(pow(AcX,2) + pow(AcZ,2)))*RAD2DEGREE;
  //Serial.print(" accel_x: "); Serial.println(comp_accel_x);
  float theta = 0; //z-axis angle

  //Complementary Filter - removes noise and drift from IMU and Gyro
  //Filtered Angle = alpha * (angle + GyroData*dt) + (1-alpha)*Accelerometer Data
  
  double dt = (t_now - last_read_time)/1000.0;
  float gyro_x = GyX * dt + last_x_angle; 
  float gyro_y = GyY * dt + last_y_angle;
  float gyro_z = GyZ * dt + last_z_angle;

  float alpha = 0.95;
  float cf_x = alpha * gyro_x + (1.0 - alpha)*comp_accel_x;
  float cf_y = alpha * gyro_y + (1.0 - alpha)*comp_accel_y;
  float cf_z = gyro_z;

  set_last_read_angle_data(t_now, cf_x, cf_y, cf_z);

  Serial.print(cf_x, 2);
  Serial.print(F(","));
  Serial.print(cf_y, 2);
  Serial.print(F(","));
  Serial.print(cf_z, 2);
  Serial.println(F(""));
  
  //PID to control motor output

  delay(100);
  }
  
