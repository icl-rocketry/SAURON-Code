// start logging and sending telemetry and errors full force (10Hz)
// check for landing, then go to landing leg deploy

#include "Arduino.h"
#include "state.h"
#include "stateMachine.h"

#ifndef FLIGHT_H
#define FLIGHT_H

class flight : public State
{
public:
    flight(stateMachine *sm);
    void initialise();
    State *update();
    void exitState();

private:
    float xAcc, yAcc, zAcc;
    float xGyro, yGyro, zGyro;
    float xMag, yMag, zMag;
    float PrevHeading{0};
    float DiffHeading;
    uint32_t filterPrevTime{0};
    uint32_t stepperPrevTime{0};
};

#endif