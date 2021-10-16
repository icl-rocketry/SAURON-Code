#include "preLaunch.h"
#include "flight.h"
#include "madgwickConfig.h"

preLaunch::preLaunch(stateMachine *sm) : State(sm)
{
}

void preLaunch::initialise()
{
  Serial.println("PreLaunch state");
  _sm->imu.calibrate(true);

  ACCEL_THRESHOLD = 1.0;
};

<<<<<<< HEAD
State *preLaunch::update()
{
  float zAcc = 0;

  if (_sm->imu.accelAvailable())
  {
    _sm->imu.readAccel();
    //float xAcc = _sm -> imu.calcAccel(imu.ax);
    //float yAcc = _sm -> imu.calcAccel(imu.ay);
    zAcc = _sm->imu.calcAccel(_sm->imu.az);
  }

  _sm->filter.reset();
  // if (zAcc > ACCEL_THRESHOLD)
  // {
  //   count_over_threshold += 1;
  // }
  // else
  // {
  //   count_over_threshold = 0;
  // }

  // if (count_over_threshold > 10)
  // {
  //   return new flight(_sm);
  // }
  // else
  // {
  //   return this;
  // }
  return new flight(_sm);
  delay(500);
=======
State* preLaunch::update() {
    float zAcc = 0;
    static float accelAvg = 0;
    static uint8_t counter = 0;
    static bool reading = false;

    _sm -> filter.reset();

    if (_sm -> imu.accelAvailable())
    {
      _sm -> imu.readAccel();
      //float xAcc = _sm -> imu.calcAccel(imu.ax);
      //float yAcc = _sm -> imu.calcAccel(imu.ay);
      zAcc = _sm -> imu.calcAccel(_sm -> imu.az); //TODO: use calibrated?
    }

    if (reading ==  true) {
      accelAvg += zAcc;
      counter++;
    } else if (zAcc > ACCEL_THRESHOLD) {
      counter = 0;
      reading = true;
    }

    if (counter >= AVG_CTR) {
      counter = 0;
      reading = false;
      if (accelAvg/AVG_CTR > ACCEL_THRESHOLD) {
        return new flight(_sm);
      } else {
        return this;
      }
    } else {
      return this;
    }
>>>>>>> 4077fbb (Added acceleration triggering)
};

void preLaunch::exitState(){

};