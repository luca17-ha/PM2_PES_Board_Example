#pragma once
#include "DCMotor.h"
#include "ValveCharacterization.h"
#include "ValveController.h"


class Valve {
public:
    Valve(DCMotor& motor, float Ts = 0.01f);
    
    void setOpen(float percent);
    float getPosition() const;
    
    void startCharacterization();
    void start(float operating_point = 0.0f);
    void quickCharacterize(Odometer& odometer, SDLogger& sd_logger);
    void startClosedLoop(float target_speed);
    void update(float measured_speed);
    bool isRunning() const;
    float getTargetSpeed() const { return target_speed_; }
    
private:
    DCMotor& motor_;
    ValveCharacterization char_;
    ValveController controller_;
    enum { MANUAL, CHARACTERIZATION, CLOSED_LOOP } mode_;
    float target_speed_ = 0.0f;
    
    float constrain(float val, float min, float max);
};