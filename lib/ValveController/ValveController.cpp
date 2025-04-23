#include "ValveController.h"

ValveController::ValveController(float Ts) : Ts_(Ts) {
    // Initialize with conservative defaults
    pid_.setup(-0.3f, -0.1f, -0.05f,  // P, I, D
               0.01f,  // tau_f (derivative filter)
               0.05f,  // tau_ro (roll-off)
               Ts,     // Sampling time
               0.0f,   // uMin
               1.0f);  // uMax
}

void ValveController::tune(float Kp, float Ki, float Kd) {
    // Maintain the same filter time constants when tuning
    pid_.setup(Kp, Ki, Kd, 
               0.01f,  // Keep original tau_f
               0.05f,  // Keep original tau_ro
               Ts_,    // Use stored Ts
               0.0f, 1.0f);  // Valve limits
}

float ValveController::update(float error) {
    return pid_.update(error);  // Direct error input
}