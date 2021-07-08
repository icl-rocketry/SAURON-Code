// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <BasicStepperDriver.h> // https://github.com/laurb9/StepperDriver
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include "MadgwickAHRS.h"

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


LSM9DS1 imu;

float beta(0.1f);
float dt(0.001f);

// initialize a Madgwick filter:
Madgwick filter(beta,dt);

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

int prev_time;

#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

 //definitions
 float xAcc, yAcc, zAcc;
 float xGyro, yGyro, zGyro;
 float xMag, yMag, zMag;
 float roll, pitch, heading;

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
  // Update the sensor values whenever new data is available
  if (millis()-prev_time > dt){
    prev_time = millis();
    if ( imu.gyroAvailable() )
    {
        imu.readGyro();
        xGyro = imu.calcGyro(imu.gx);
        yGyro = imu.calcGyro(imu.gy);
        zGyro = imu.calcGyro(imu.gz);
        
    }
    if ( imu.accelAvailable() )
    {
        imu.readAccel();
        xAcc = imu.calcAccel(imu.ax);
        yAcc = imu.calcAccel(imu.ay);
        zAcc = imu.calcAccel(imu.az);
        
    }
     if ( imu.magAvailable() )
    {
        imu.readMag();
        xMag = imu.calcAccel(imu.mx);
        yMag = imu.calcAccel(imu.my);
        zMag = imu.calcAccel(imu.mz);
        
    }

    
    filter.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, xMag, yMag, zMag);
    //filter.updateIMU();

        // print the heading, pitch and roll
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();

        stepper.rotate(heading);
  }
}
