#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
public:
    // Noise States für Mehrschichten-Ansatz
    enum NoiseState {
        NOISE_LOW,      // σ ≈ 0.0017 m/s² (Standard)
        NOISE_MEDIUM,   // σ ≈ 0.01 m/s²
        NOISE_HIGH      // σ ≈ 0.05 m/s²
    };

    KalmanFilter(float dt, float base_process_noise, float base_measurement_noise);
    
    float update(float measurement);
    float getFilteredValue() const;

    static constexpr float MIN_R = 0.001f;  // Mindest-Messrauschen (m²/s⁴)
    static constexpr float MIN_Q = 0.001f;  // Mindest-Prozessrauschen (m/s²)
    
    // Adaptive Methoden
    void setNoiseState(NoiseState state);
    void autoDetectNoise(float current_measurement);

    // Prädiktion
    float getLastPrediction() const { return x_pred_; }

private:
    float dt_;
    float x_;
    float p_;
    
    // Basisrauschen
    float q_base_;
    float r_base_;
    
    // Aktuelle multiplikative Faktoren
    float q_factor_;
    float r_factor_;
    
    // Noise Detection
    float moving_avg_abs_noise_ = 0;
    const float alpha_ = 0.1f; // EMA-Faktor
    float x_pred_; // Zwischenspeicher für Prädiktion

    bool first_update_ = true;  // Initialisierung beim ersten Update
};

#endif // KALMAN_FILTER_H