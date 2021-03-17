
// Include necessary libraries
#include <BasicStepperDriver.h> // https://github.com/laurb9/StepperDriver
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h> // https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library


// Motor steps per revolution. Our stepper is 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 550

// All the wires needed for full functionality
#define DIR 28
#define STEP 29


// Since microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 16

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

// definitions 

float pitch_g = 0;
float roll_g = 0;
float yaw_g = 0;

float heading;
float roll_a;
float pitch_a;
float Mx;
float My;

float yaw_f = 0;


// Timers
unsigned long timer = 0;
float timeStep = 0.02;

// Define LSM9DS1 object
LSM9DS1 imu;


// Example I2C Setup - Change depending on setups
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW


// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. This is taken from somewhere in UK
#define DECLINATION -1.170




void setup() 
{
  Serial.begin(115200);

  Wire.begin();

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
   
   // set stepper motor speed and microsteps
    stepper.setRPM(100);
    stepper.setMicrostep(MICROSTEPS);
}

void loop() 
{

   timer = millis();
   
   // insert the chosen way to calculate yaw rate 
   imu.readGyro();
   
   yaw_g = yaw_g + imu.calcGyro(imu.gz)*timeStep;
   roll_g = roll_g + imu.calcGyro(imu.gx)*timeStep;
   pitch_g = pitch_g + imu.calcGyro(imu.gy)*timeStep;


   // Accelerometer and magnetometer data can also be used. This can be used to compare difference between methods. Tilt compensation for the headign angle is also added
   imu.readAccel();
   imu.readMag();

   float ay = imu.calcAccel(imu.ay)
   float az = imu.calcAccel(imu.az)
   float ax = imu.calcAccel(imu.ax)
   float mx = imu.calcMag(imu.mx)
   float my = imu.calcMag(imu.my)

   roll_a = atan2(ay, az);
   pitch_a = atan2(-ax, sqrt(ay * ay + az * az));
  
   Mx = mx*cos(pitch_a) + mz*sin(pitch_a);
   My = mx*sin(roll_a)*sin(pitch_a) + my*cos(roll_a) - mz*sin(roll_a)*cos(pitch_a)

   heading = atan(-My/Mx);
    
   heading -= DECLINATION * PI / 180;
  
   if (heading > PI) heading -= (2 * PI);
   else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch_a *= 180.0 / PI;
  roll_a  *= 180.0 / PI;

// accelerometer performs the best with low frequency while gyroscope
performs the best with high frequency. A complimentary filter can be used. The gains can be adjusted with experiments

yaw_f = 0.98*(yaw_f + imu.calcGyro(imu.gz)*timeStep) + 0.02*heading;


   
}   
    delay((timeStep*1000) - (millis() - timer));
