


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

// define yaw pitch and roll
float pitch = 0;
float roll = 0;
float yaw = 0;

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
   
   yaw = yaw + imu.calcGyro(imu.gz)*timeStep;
   roll = roll + imu.calcGyro(imu.gx)*timeStep;
   pitch = pitch + imu.calcGyro(imu.gy)*timeStep;

   // Output in degrees
   Serial.print(" Outputs obtained using Gyroscope readings");
   Serial.print(" Pitch = ");
   Serial.print(pitch);
   Serial.print(" Roll = ");
   Serial.print(roll);  
   Serial.print(" Yaw = ");
   Serial.println(yaw);

   // making the stepper rotate by relative position
   stepper.rotate(imu.calcGyro(imu.gz)*timeStep)






   // As a comparison, accelerometer and magnetometer data can be used. This can be used to compare difference between methods
   imu.readAccel();
   imu.readMag();

   float ay = imu.calcAccel(imu.ay)
   float az = imu.calcAccel(imu.az)
   float ax = imu.calcAccel(imu.ax)
   float mx = imu.calcMag(imu.mx)
   float my = imu.calcMag(imu.my)

   float roll2 = atan2(ay, az);
   float pitch2 = atan2(-ax, sqrt(ay * ay + az * az));
  
   float heading;
   if (my == 0)
    heading = (mx < 0) ? PI : 0;
   else
    heading = atan2(mx, my);
    
   heading -= DECLINATION * PI / 180;
  
   if (heading > PI) heading -= (2 * PI);
   else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch2 *= 180.0 / PI;
  roll2  *= 180.0 / PI;

  // Printing results as comparison
  Serial.print(" Outputs obtained using Accelerometer and Magnetometer readings ")
  Serial.print(" Pitch = ");
  Serial.print(heading);
  Serial.print(" Roll = ");
  Serial.print(roll2);  
  Serial.print(" Yaw = ");
  Serial.println(yaw2);

   
}   
    delay((timeStep*1000) - (millis() - timer));
