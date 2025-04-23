#include "KalmanFilter.h"
#include <cmath>

// Konstruktor mit Basiswerten
KalmanFilter::KalmanFilter(float dt, float base_process_noise, float base_measurement_noise)
    : dt_(dt), x_(0), p_(1.0f),
      // NEU: Basis-Q kleiner wählen (z. B. 0.01 statt 0.1)
      q_base_(0.01f),  // Vorher: base_process_noise
      r_base_(base_measurement_noise),
      q_factor_(1.0f), r_factor_(1.0f),
      first_update_(true) {}  // NEU: Erstinitialisierung

// Haupt-Update mit adaptivem Rauschen
float KalmanFilter::update(float measurement) {
    // NEU: Beim ersten Update direkt Messwert übernehmen
    if (first_update_) {
        x_ = measurement;
        p_ = r_base_ * r_factor_;
        first_update_ = false;
        return x_;
    }

    // Prädiktion
    x_pred_ = x_;
    float innov = measurement - x_pred_;
    
    // NEU: Dynamische Rauschanpassung mit Mindestwert
    autoDetectNoise(innov);
    float effective_r = std::fmax(r_base_ * r_factor_, MIN_R);  // R nicht zu klein
    float effective_q = std::fmax(q_base_ * q_factor_, MIN_Q);  // Q nicht zu klein

    // Rest unverändert
    float p_pred = p_ + effective_q * dt_ * dt_;
    float k = p_pred / (p_pred + effective_r);
    x_ = x_pred_ + k * innov;
    p_ = (1 - k) * p_pred;
    
    return x_;
}

// Manuelle Rauschzustand-Setzung
void KalmanFilter::setNoiseState(NoiseState state) {
    switch(state) {
        case NOISE_LOW:
            q_factor_ = 1.0f;   // Q = 0.01 m/s² (vorher: 0.1)
            r_factor_ = 1.0f;   // R = Basiswert (z. B. 0.001)
            break;
        case NOISE_MEDIUM:
            q_factor_ = 3.0f;   // Q = 0.03 m/s² (vorher: 0.3)
            r_factor_ = 10.0f;  // R = 10x Basiswert
            break;
        case NOISE_HIGH:
            q_factor_ = 10.0f;  // Q = 0.1 m/s² (vorher: 1.0)
            r_factor_ = 50.0f;  // R = 50x Basiswert
            break;
    }
}

// Automatische Rauscherkennung
void KalmanFilter::autoDetectNoise(float innov) {
    // NEU: Schnellerer EMA-Filter (alpha = 0.3 statt 0.1)
    moving_avg_abs_noise_ = 0.3f * fabs(innov) + 0.7f * moving_avg_abs_noise_;
    
    // NEU: Schwellwerte mit Hysterese
    static NoiseState last_state = NOISE_LOW;
    if (moving_avg_abs_noise_ > 0.05f) {
        last_state = NOISE_HIGH;
    } 
    else if (moving_avg_abs_noise_ > 0.02f && last_state != NOISE_HIGH) {
        last_state = NOISE_MEDIUM;
    } 
    else if (moving_avg_abs_noise_ < 0.01f) {
        last_state = NOISE_LOW;
    }
    setNoiseState(last_state);
}

float KalmanFilter::getFilteredValue() const {
    return x_;
}