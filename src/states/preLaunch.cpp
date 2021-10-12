#include "preLaunch.h"
#include "flight.h"
#include "madgwickConfig.h"

preLaunch::preLaunch(stateMachine* sm):
State(sm)
{}

void preLaunch::initialise() {
  Serial.println("PreLaunch state");
  _sm -> imu.calibrate(true);
};

State* preLaunch::update() {
    float zAcc = 0;

    if (_sm -> imu.accelAvailable())
    {
      _sm -> imu.readAccel();
      //float xAcc = _sm -> imu.calcAccel(imu.ax);
      //float yAcc = _sm -> imu.calcAccel(imu.ay);
      zAcc = _sm -> imu.calcAccel(_sm -> imu.az); //TODO: use calibrated?
    }

    //delay(10000);
    _sm -> filter.reset();
    //if (zAcc > ACCEL_THRESHOLD) {
      return new flight(_sm); 
    //} else {
        //return this;
    //}
};

void preLaunch::exitState() {

};