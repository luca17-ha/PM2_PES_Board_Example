#include "ValveCharacterization.h"



//legacy
ValveCharacterization::ValveCharacterization(DCMotor& motor, float Ts, TuningCallback cb) 
    : motor_(motor), Ts_(Ts), onTuningComplete_(cb) {}

void ValveCharacterization::start(float op_point) {
    motor_.setRotation(op_point);
    logging_timer_.reset();   // timer auf null setzen
    logging_timer_.start();   // timer starten
    time_previous_us_ = 0;
    running_ = true;
}


bool ValveCharacterization::isRunning() const {
    return running_;
}

//legacy
float ValveCharacterization::average(const std::array<float, 100>& arr, size_t start, size_t n) {
    float sum = 0;
    for(size_t i = start; i < start + n; i++) {
        sum += arr[i];
    }
    return sum / n;
}



int counter_value = 0;

void ValveCharacterization::quickCharacterize(Odometer& odometer, SDLogger& sd_logger) {
    running_ = true;
    // measure delta time
    const long long time_us = logging_timer_.elapsed_time().count();
    const long long dtime_us = time_us - time_previous_us_;
    time_previous_us_ = time_us;

    printf("Timer started: %lld\n", time_us);
    printf("time_us: %lld | delta: %lld us\n", time_us, dtime_us);

    sd_logger.write(dtime_us);
    sd_logger.write(odometer.getSpeedMPS());
    sd_logger.write((float)counter_value);  // Use counter_value here
    sd_logger.send();
    counter_value++;  // Increment counter_value  
    if (counter_value % 50 == 0) {
        printf("counter_value: %d\n", counter_value);
    }

    //logic to decied when to start step and end characterization
    if (counter_value > 500) {
        counter_value = 0;
        motor_.setRotation(0.0f);
        printf("leaving characterization state\n");
        running_ = false;
    } else if (counter_value > 250) {
        //start step
        motor_.setRotation(0.5f);
    }
}


//legacy
float ValveCharacterization::findTimeTo63Percent(const std::array<float, 100>& speeds, float Ts) {
    const float steady = speeds.back();
    const float target = speeds[0] + 0.63f * (steady - speeds[0]);
    for(size_t i = 0; i < speeds.size(); i++) {
        if(speeds[i] >= target) return i * Ts;
    }
    return 0.1f; // Default if not found
}
//legacy
void ValveCharacterization::update(float measured_speed) {
    // Original GPA-based characterization if needed
    // (Implement similarly to quickCharacterize but using GPA)
    running_ = false;
}