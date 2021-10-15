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

  uint32_t prevTime = millis();
  _sm->imu.readGyro();
  // Serial.print("readGyro: " + String(millis() - prevTime) + "\n"); // < 4 millis
  prevTime = millis();
  xGyro = _sm->imu.calcGyro(_sm->imu.gx);
  yGyro = -_sm->imu.calcGyro(_sm->imu.gy);
  zGyro = _sm->imu.calcGyro(_sm->imu.gz);
  // Serial.print("calcGyro  " + String(millis() - prevTime)+ "\n"); // < 4 millis

  prevTime = millis();
  _sm->imu.readAccel();
  // Serial.print("readAccel  " + String(millis() - prevTime) + "\n"); // < 4 millis

  prevTime = millis();
  xAcc = _sm->imu.calcAccel(_sm->imu.ax);
  yAcc = -_sm->imu.calcAccel(_sm->imu.ay);
  zAcc = _sm->imu.calcAccel(_sm->imu.az);
  // Serial.print("calcAccel  " + String(millis() - prevTime) + "\n"); // < 3 millis

  prevTime = millis();
  _sm->imu.readMag();
  // Serial.print("readMag  " + String(millis() - prevTime) + "\n"); // < 4 millis

  // Pre-calibration
  // xMag = -_sm->imu.calcAccel(_sm->imu.mx);
  // yMag = -_sm->imu.calcAccel(_sm->imu.my);
  // zMag = _sm->imu.calcAccel(_sm->imu.mz);

  prevTime = millis();
  subbed_imu_mx = (_sm->imu.mx - 0.46288029784302);
  subbed_imu_my = (_sm->imu.my - 1.01246637335654);
  subbed_imu_mz = (_sm->imu.mz + 0.72659380318724);

  cal_imu_mx = 1.391743393369 * subbed_imu_mx - 0.091636916484 * subbed_imu_my - 0.014574125665 * subbed_imu_mz;
  cal_imu_my = -0.916369164849 * subbed_imu_mx + 1.2520273342610 * subbed_imu_my + 0.076717134777 * subbed_imu_mz;
  cal_imu_mz = -0.014574125666 * subbed_imu_mx + 0.0767171347770 * subbed_imu_my + 1.310715522801 * subbed_imu_mz;
  // Serial.print("caliBrate  " + String(millis() - prevTime) + "\n"); // < 4 millis

  prevTime = millis();
  xMag = -_sm->imu.calcAccel(cal_imu_mx);
  yMag = -_sm->imu.calcAccel(cal_imu_my);
  zMag = -_sm->imu.calcAccel(cal_imu_mz);
  // Serial.print("calcMag  " + String(millis() - prevTime) + "\n"); // < 4 millis

  // Get actual dt
  uint32_t actualDt = millis() - filterPrevTime;
  filterPrevTime = millis();
  _sm->filter.setDeltaT((float)actualDt / 1000.0);

  prevTime = millis();
  _sm->filter.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, xMag, yMag, zMag);
  // Serial.print("update  " + String(millis() - prevTime) + "\n"); // < 8 millis

  prevTime = millis();
  _sm->filter.computeAngles();
  // Serial.print("computeAngles  " + String(millis() - prevTime) + "\n"); // < 5 millis

  // print the heading, pitch and roll
  float roll = _sm->filter.getRoll();
  float pitch = _sm->filter.getPitch();
  float heading = _sm->filter.getYaw();

  if ((millis() - stepperPrevTime) > .1)
  {
    // Heading wrap-around error-catching
    if (heading * PrevHeading < 0 && heading > 2)
    {
      // Wrapping from negative to positive
      DiffHeading = (heading - PrevHeading - 2 * PI) * 57.29577951;
    }
    else if (heading * PrevHeading < 0 && heading < -2)
    {
      // Wrapping from positive to negative
      DiffHeading = (heading - PrevHeading + 2 * PI) * 57.29577951;
    }
    else
    {
      DiffHeading = (heading - PrevHeading) * 57.29577951;
    }

    _sm->stepper.rotate(-DiffHeading * 0.625); // gear ratio = 20/32 = 0.625
    if (_sm->stepper.calcStepsForRotation(-DiffHeading * 0.625) != 0)
    {
      PrevHeading = heading;
    }
    stepperPrevTime = millis();
  }

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