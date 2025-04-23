#pragma once
#include "PIDCntrl.h"

class ValveController {
public:
    ValveController(float Ts = 0.01f);
    
    void tune(float Kp, float Ki, float Kd);
    float update(float error);  // Takes error directly
    
    float getTs() const { return Ts_; }  // Add this to track Ts
    
private:
    PIDCntrl pid_;
    float Ts_;  // Store sampling time locally
};