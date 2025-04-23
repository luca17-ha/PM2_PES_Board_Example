#include "Valve.h"


Valve::Valve(DCMotor& motor, float Ts) : 
    motor_(motor),
    char_(motor, Ts, [this](float Kp, float Ki, float Kd) {
        controller_.tune(Kp, Ki, Kd);
    }),
    controller_(Ts),
    mode_(MANUAL),
    target_speed_(0.0f)
{
    // Initialize safe defaults
    controller_.tune(-0.8f, -0.1f, -0.05f);
}

void Valve::setOpen(float percent) {
    percent = constrain(percent, 0.0f, 1.0f);
    motor_.setRotation(percent);
    mode_ = MANUAL;
}

float Valve::getPosition() const {
    return motor_.getRotation();
}

// not implemented yet
void Valve::startCharacterization() {
    char_.start(0.5f);
    mode_ = CHARACTERIZATION;
}

bool Valve::isRunning() const {
    return char_.isRunning();
}

void Valve::start(float op_point) {
    char_.start(op_point);            // this actually starts the characterization
}

void Valve::quickCharacterize(Odometer& odometer, SDLogger& sd_logger) {
    char_.quickCharacterize(odometer, sd_logger);
    mode_ = MANUAL;
}

void Valve::startClosedLoop(float target_speed) {
    target_speed_ = target_speed;
    mode_ = CLOSED_LOOP;
}



void Valve::update(float measured_speed) {
    switch(mode_) {
        case CHARACTERIZATION: 
            char_.update(measured_speed);
            if(!char_.isRunning()) {
                mode_ = MANUAL;
            }
            break;

        case CLOSED_LOOP: {
            float error = target_speed_ - measured_speed;
            printf("error: %.3f", error);
            float valve_cmd = 1.0f - controller_.update(error);
            printf("valve cmd: %.3f", valve_cmd);
            motor_.setRotation(constrain(valve_cmd, 0.0f, 1.0f));
            //motor_.setRotation(2.0f);
            break;
        }

        case MANUAL:
            if(!char_.isRunning()) {
                printf("characterization finished\n");
                printf("check sd-card for results\n");
                motor_.setRotation(0.0f);
            }
        default:
            break;
    }
}

float Valve::constrain(float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}