/* 
Code used to process states, and the transitions between them

Written by the Electronics team, Imperial College London Rocketry
*/

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include "MadgwickAHRS.h"
#include <BasicStepperDriver.h> // https://github.com/laurb9/StepperDriver

class State;

class stateMachine {
  public:
    stateMachine();
    void initialise(State* initStatePtr);
    void update();
    void exitState();
    void changeState(State* newStatePtr);

    LSM9DS1 imu;
    Madgwick filter;
    BasicStepperDriver stepper;
    
    uint64_t prev_time;
    
  private:
    State* _currStatePtr;
};

#endif
