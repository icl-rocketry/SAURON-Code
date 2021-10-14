// Motor steps per revolution. Our stepper is 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 550

// All the wires needed for full functionality
#define DIR A0
#define STEP A1

// Since microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 16