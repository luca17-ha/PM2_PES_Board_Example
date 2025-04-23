#ifndef ODOMETER_H
#define ODOMETER_H

#include "mbed.h"
#include "EncoderCounter.h"

class Odometer {
public:
    /**
     * @param encoder Existing EncoderCounter instance
     * @param wheel_diameter_mm Wheel diameter in millimeters
     * @param counts_per_rev Encoder counts per full revolution
     * @param update_interval_ms Speed calculation interval (default 50ms)
     */
    Odometer(EncoderCounter& encoder, float wheel_diameter_mm, uint16_t counts_per_rev = 20, uint16_t update_interval_ms = 20);
    
    void update();  // Call this regularly
    void reset();
    
    // Measurement getters
    int32_t getCount() const;
    float getDistanceMM() const;
    float getDistanceM() const;
    float getSpeedMPS() const;  // Meters per second
    float getSpeedRPM() const;  // Revolutions per minute
    
    // Configuration
    void setWheelDiameter(float diameter_mm);
    void setCountsPerRev(uint16_t counts);

private:
    EncoderCounter& encoder_;
    Timer speed_timer_;
    
    // Configuration
    float wheel_diameter_mm_;
    float wheel_circumference_mm_;
    float counts_per_mm_;
    uint16_t counts_per_rev_;
    uint16_t update_interval_ms_;
    
    // Runtime data
    int32_t last_count_ = 0;
    float current_speed_mps_ = 0.0f;
    
    void calculateDerivedParams();
};

#endif