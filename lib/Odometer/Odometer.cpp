#include "Odometer.h"

Odometer::Odometer(EncoderCounter& encoder, float wheel_diameter_mm, uint16_t counts_per_rev, uint16_t update_interval_ms) :
    encoder_(encoder),
    wheel_diameter_mm_(wheel_diameter_mm),
    counts_per_rev_(counts_per_rev),
    update_interval_ms_(update_interval_ms) {
    
    calculateDerivedParams();
    speed_timer_.start();
}

void Odometer::calculateDerivedParams() {
    wheel_circumference_mm_ = 3.14159265358979323846f * wheel_diameter_mm_;
    counts_per_mm_ = counts_per_rev_ / wheel_circumference_mm_;
}

void Odometer::update() {
    if (speed_timer_.elapsed_time().count() >= update_interval_ms_ * 1000) {
        int32_t current_count = encoder_.read();
        int32_t count_diff = current_count - last_count_;
        float time_elapsed = speed_timer_.elapsed_time().count() / 1e6f;
        
        current_speed_mps_ = (count_diff / counts_per_mm_) / (time_elapsed * 1000.0f);
        last_count_ = current_count;
        speed_timer_.reset();
    }
}

void Odometer::reset() {
    encoder_.reset();
    last_count_ = 0;
    current_speed_mps_ = 0.0f;
    speed_timer_.reset();
}

// Getters implementation
int32_t Odometer::getCount() const { return encoder_.read(); }
float Odometer::getDistanceMM() const { return encoder_.read() / counts_per_mm_; }
float Odometer::getDistanceM() const { return getDistanceMM() / 1000.0f; }
float Odometer::getSpeedMPS() const { return current_speed_mps_; }
float Odometer::getSpeedRPM() const { 
    return (current_speed_mps_ * 60.0f) / (wheel_circumference_mm_ / 1000.0f); 
}

void Odometer::setWheelDiameter(float diameter_mm) {
    wheel_diameter_mm_ = diameter_mm;
    calculateDerivedParams();
}

void Odometer::setCountsPerRev(uint16_t counts) {
    counts_per_rev_ = counts;
    calculateDerivedParams();
}