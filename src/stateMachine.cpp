/* 
Code used to process states, and the transitions between them

Written by the Electronics team, Imperial College London Rocketry
*/

#include "Arduino.h"
#include "stateMachine.h"
#include "states/state.h"
#include "stepperConfig.h"
#include "madgwickConfig.h"

stateMachine::stateMachine() : imu(),
                               filter(beta, dt),
                               stepper(MOTOR_STEPS, DIR, STEP),
                               prev_time(0)
                               {};

void stateMachine::initialise(State *initStatePtr)
{

  Serial.begin(115200);

  Serial.println("Initialising IMU and Stepper...");

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will "
                   "work for an out of the box LSM9DS1 "
                   "Breakout, but may need to be modified "
                   "if the board jumpers are.");
    while (1)
      ;
  }

  // set stepper motor speed and microsteps
  stepper.setRPM(100);
  stepper.setMicrostep(MICROSTEPS);

  // Initialise the classes
  changeState(initStatePtr);
}

void stateMachine::update()
{
  if (millis() - prev_time > dt) // Only update if dt has passed since last update
  {
    prev_time = millis();

    State *newStatePtr = _currStatePtr->update();

    if (newStatePtr != _currStatePtr)
    {
      exitState();
      changeState(newStatePtr);
    }
  }
}

void stateMachine::changeState(State *newStatePtr)
{
  // Delete old state instance and change to new one
  Serial.println("Changing State...");
  delete _currStatePtr;
  _currStatePtr = newStatePtr;
  _currStatePtr->initialise();
}

void stateMachine::exitState()
{
  _currStatePtr->exitState();
}