#include "flight.h"

flight::flight(stateMachine *sm) : State(sm)
{
}

void flight::initialise()
{
  Serial.println("Flight state");
};

State *flight::update()
{

  _sm->imu.readGyro();
  xGyro = _sm->imu.calcGyro(_sm->imu.gx);
  yGyro = -_sm->imu.calcGyro(_sm->imu.gy);
  zGyro = _sm->imu.calcGyro(_sm->imu.gz);

  _sm->imu.readAccel();
  xAcc = _sm->imu.calcAccel(_sm->imu.ax);
  yAcc = -_sm->imu.calcAccel(_sm->imu.ay);
  zAcc = _sm->imu.calcAccel(_sm->imu.az);

  _sm->imu.readMag();

  // Pre-calibration
  // xMag = -_sm->imu.calcAccel(_sm->imu.mx);
  // yMag = -_sm->imu.calcAccel(_sm->imu.my);
  // zMag = _sm->imu.calcAccel(_sm->imu.mz);

  double subbed_imu_mx = (_sm->imu.mx - 0.46288029784302);
  double subbed_imu_my = (_sm->imu.my - 1.01246637335654);
  double subbed_imu_mz = (_sm->imu.mz + 0.72659380318724);

  double cal_imu_mx = 1.391743393369 * subbed_imu_mx - 0.091636916484 * subbed_imu_my - 0.014574125665 * subbed_imu_mz;
  double cal_imu_my = -0.916369164849 * subbed_imu_mx + 1.2520273342610 * subbed_imu_my + 0.076717134777 * subbed_imu_mz;
  double cal_imu_mz = -0.014574125666 * subbed_imu_mx + 0.0767171347770 * subbed_imu_my + 1.310715522801 * subbed_imu_mz;

  xMag = -_sm->imu.calcAccel(cal_imu_mx);
  yMag = -_sm->imu.calcAccel(cal_imu_my);
  zMag = -_sm->imu.calcAccel(cal_imu_mz);

  // Get actual dt
  uint64_t actualDt = millis() - _sm->prev_time;
  _sm->filter.setDeltaT((float)actualDt / 1000.0);
  _sm->prev_time = millis();

  _sm->filter.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, xMag, yMag, zMag);

  _sm->filter.computeAngles();

  // print the heading, pitch and roll
  float roll = _sm->filter.getRoll();
  float pitch = _sm->filter.getPitch();
  float heading = _sm->filter.getYaw();

  //_sm -> stepper.rotate(heading*57.29577951);
  Serial.print(roll * 57.29577951);
  Serial.print(" | ");
  Serial.print(pitch * 57.29577951);
  Serial.print(" | ");
  Serial.println(heading * 57.29577951);

  // Never leave state
  return this;
}

void flight::exitState(){

};