#pragma once
#include "mbed.h"
#include <cmath>
#include <functional>
#include <array>
#include "DCMotor.h"
#include "Odometer.h"
#include "SDLogger.h"
#include <cstdio>
#include <chrono>
using namespace std::chrono;


class ValveCharacterization {
public:
    using TuningCallback = std::function<void(float Kp, float Ki, float Kd)>;
    
    ValveCharacterization(DCMotor& motor, float Ts, TuningCallback cb);
    
    void start(float operating_point = 0.0f);
    bool isRunning() const;
    void update(float measured_speed);
    void quickCharacterize(Odometer& odometer, SDLogger& sd_logger);
    
private:
    DCMotor& motor_;
    float Ts_;
    TuningCallback onTuningComplete_;
    Timer logging_timer_;  // Jetzt als Member
    long long time_previous_us_{0};
    bool running_ = false;
    
    float readSpeedSensor_();  // legacy
    float average(const std::array<float, 100>& arr, size_t start, size_t n);
    float findTimeTo63Percent(const std::array<float, 100>& speeds, float Ts);
};