#include "flight.h"

flight::flight(stateMachine* sm):
    State(sm)
{}

void flight::initialise() {
  Serial.println("Flight state");
};

State* flight::update() {

      _sm -> imu.readGyro();
      xGyro = _sm -> imu.calcGyro(_sm -> imu.gx);
      yGyro = - _sm -> imu.calcGyro(_sm -> imu.gy);
      zGyro = _sm -> imu.calcGyro(_sm -> imu.gz);

      _sm -> imu.readAccel();
      xAcc = _sm -> imu.calcAccel(_sm -> imu.ax);
      yAcc = - _sm -> imu.calcAccel(_sm -> imu.ay);
      zAcc = _sm -> imu.calcAccel(_sm -> imu.az);

      _sm -> imu.readMag();
      xMag = - _sm -> imu.calcAccel(_sm -> imu.mx);
      yMag = - _sm -> imu.calcAccel(_sm -> imu.my);
      zMag = _sm -> imu.calcAccel(_sm -> imu.mz);


    // Get actual dt
    uint64_t actualDt = millis() - _sm->prev_time;
    _sm -> filter.setDeltaT((float)actualDt/1000.0);
    _sm -> prev_time = millis();

    _sm -> filter.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, xMag, yMag, zMag);

    _sm -> filter.computeAngles();

    // print the heading, pitch and roll
    float roll = _sm -> filter.getRoll();
    float pitch = _sm -> filter.getPitch();
    float heading = _sm -> filter.getYaw();

    //_sm -> stepper.rotate(heading*57.29577951);
    Serial.print(roll*57.29577951);
    Serial.print(" | ");
    Serial.print(pitch*57.29577951);
    Serial.print(" | ");
    Serial.println(heading*57.29577951);

    // Never leave state
    return this;
}

void flight::exitState() {

};