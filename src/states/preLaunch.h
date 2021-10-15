// Default class on startup
// initialise sensors
// Buzz buzzer to show succesful startup
// provide a slow telemetry link
// check for launch
// tighten landing legs
// Provide option for wiping and providing data on flash

#include "Arduino.h"
#include "state.h"
#include "../stateMachine.h"

#ifndef PRELAUNCH_H
#define PRELAUNCH_H

class preLaunch : public State
{
public:
    preLaunch(stateMachine *sm);
    void initialise();
    State *update();
    void exitState();

private:
    float subbed_imu_mx, subbed_imu_my, subbed_imu_mz;
    float ACCEL_THRESHOLD;
    uint32_t count_over_threshold;
};

#endif