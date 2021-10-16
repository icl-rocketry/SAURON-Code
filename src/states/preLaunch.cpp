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

};

State* preLaunch::update() {
    float zAcc = 0;
    static float accelAvg = 0;
    static uint8_t counter = 0;
    static bool reading = false;

    _sm -> filter.reset();

    Serial.print("in prelaunch ");
    if (_sm -> imu.accelAvailable())
    {
      _sm -> imu.readAccel();
      //float xAcc = _sm -> imu.calcAccel(imu.ax);
      //float yAcc = _sm -> imu.calcAccel(imu.ay);
      zAcc = _sm -> imu.calcAccel(_sm -> imu.az); //TODO: use calibrated?
    }
    Serial.println(zAcc);

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
};

void preLaunch::exitState(){

};