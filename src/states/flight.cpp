#include "flight.h"

flight::flight(stateMachine* sm):
    State(sm)
{}

void flight::initialise() {
};

State* flight::update() {
    if (_sm -> imu.gyroAvailable())
    {
      _sm -> imu.readGyro();
      xGyro = _sm -> imu.calcGyro(_sm -> imu.gx);
      yGyro = _sm -> imu.calcGyro(_sm -> imu.gy);
      zGyro = _sm -> imu.calcGyro(_sm -> imu.gz);
    }
    if (_sm -> imu.accelAvailable())
    {
      _sm -> imu.readAccel();
      xAcc = _sm -> imu.calcAccel(_sm -> imu.ax);
      yAcc = _sm -> imu.calcAccel(_sm -> imu.ay);
      zAcc = _sm -> imu.calcAccel(_sm -> imu.az);
    }
    if (_sm -> imu.magAvailable())
    {
      _sm -> imu.readMag();
      xMag = _sm -> imu.calcAccel(_sm -> imu.mx);
      yMag = _sm -> imu.calcAccel(_sm -> imu.my);
      zMag = _sm -> imu.calcAccel(_sm -> imu.mz);
    }

    // Get actual dt
    uint32_t actualDt = millis() - _sm->prev_time;
    _sm -> filter.setDeltaT(actualDt);

    _sm -> filter.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, xMag, yMag, zMag);
    //filter.update_sm -> imu();

    // print the heading, pitch and roll
    float roll = _sm -> filter.getRoll();
    float pitch = _sm -> filter.getPitch();
    float heading = _sm -> filter.getYaw();

    _sm -> stepper.rotate(heading);

    // Never leave state
    return this;
}

void flight::exitState() {

};