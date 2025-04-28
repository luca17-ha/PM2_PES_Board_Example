#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "Odometer.h"
#include "Valve.h"
#include "Chirp.h"
#include "SDLogger.h"
#include "UltrasonicSensor.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition below
                                   

// main runs as an own thread
int main()
{
    //variables
    int sleep = 0;

    // ultra sonic sensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;
    

    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    //Define Encoder Counter for linear measurement
    const float wheel_diameter_mm = 1000;

    EncoderCounter odometer_encoder(PB_ENC_A_M2, PB_ENC_B_M2);
    Odometer odometer(odometer_encoder, wheel_diameter_mm);
    odometer.update();



    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    enable_motors = 0; 

    //define Motor M1
    const float gear_ratio_M1 = 250.0f; // gear ratio * constant for valve compensation current constant: 10
    //valve compensation: rotation(1.0f) should completely close the Valve
    //valve compensation: rotation(0.0f) should completely open the Valve
    
    const float kn_M1 = 130.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, there fore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    // enable the motion planner for smooth movement
    //motor_M1.enableMotionPlanner();
    // limit max. velocity to half physical possible velocity
    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 1.0f);




    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                        // the main task will run 50 times per second

    //define Valve controller
    PIDCntrl pid;  // Your PID controller

/*
    // Define tuning callback (updates PID gains)
    auto onTuningComplete = [&pid](float Kp, float Ki, float Kd) {
        pid.setCoeff_P(Kp);
        pid.setCoeff_I(Ki);
        pid.setCoeff_D(Kd);
    };
*/

    // sd card logger
    SDLogger sd_logger(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);

    //valve => nozzle to regulate airflow
    Valve valve(motor_M1, (float)main_task_period_ms / 1000);  // 20ms sampling time


    //printf("Starting valve characterization...\n");
    //valve.quickCharacterize(odometer);  // Takes ~10 seconds
    //printf("Tuning complete!\n");
    


    // led on nucleo board
    DigitalOut user_led(LED1);

    enum RobotState {
        INITIAL,
        PID_CHARACTERIZATION,
        INIT_EXECUTION,
        EXECUTION,
        SLEEP,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

    //Timer which is used for logging during EXECUTION
    Timer logging_timer;
    long long time_previous_us = 0; // used for data logging during execution

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // start timer
    main_task_timer.start();



    // this loop will run forever
    while (true) {
        main_task_timer.reset();


        if (do_execute_main_task) {
            




        switch(robot_state) {
            case RobotState::INITIAL: {
                // visual feedback that the main task is executed, setting this once would actually be enough
                user_led = 1;
                enable_motors = 1;
                robot_state = INIT_EXECUTION; //jump to characterization or execution
                break; 
            }
            case RobotState::PID_CHARACTERIZATION: {
                static bool has_started = false;
                if (!has_started) {
                    printf("Starting valve characterization...\n");
                    valve.start(0.0f);
                    has_started = true;
                }
                
                valve.quickCharacterize(odometer, sd_logger);

                if (!valve.isRunning()) {
                    printf("Tuning complete!\n");
                    has_started = false;
                    robot_state = SLEEP;
                }
                break; 
            }
            case RobotState::INIT_EXECUTION: {
                logging_timer.reset();
                logging_timer.start();
                time_previous_us = 0;
                robot_state = EXECUTION;
                break;
            }
            case RobotState::EXECUTION: {
                //this could be only set once if speed target doesnt change
                valve.startClosedLoop(2.0f);

                //odometer update and output
                odometer.update();
                
                //printf("Count: %d32 | ", odometer.getCount());
                //printf("Distance: %.2f m | ", odometer.getDistanceM());
                //printf("Speed: %.2f m/s | ", odometer.getSpeedMPS());
                //printf("%.1f RPM\n", odometer.getSpeedRPM());
 
                valve.update(odometer.getSpeedMPS());
                /*
                printf("Speed: %.2f | Valve: %.1f%%\n", 
                    odometer.getSpeedMPS(), 
                    valve.getPosition()*100); 
                */

                    const long long time_us = logging_timer.elapsed_time().count();
                    const long long dtime_us = time_us - time_previous_us;
                    time_previous_us = time_us;

                    printf("Timer started: %lld\n", time_us);
                    printf("time_us: %lld | delta: %lld us\n", time_us, dtime_us);

                    sd_logger.write(dtime_us);
                    sd_logger.write(odometer.getSpeedMPS());
                    sd_logger.write(us_sensor.read());
                    sd_logger.write(odometer.getDistanceM());
                    sd_logger.send();
                break; 
            }
            case RobotState::EMERGENCY: {
                printf("us_sensor_cm: %f\n", us_sensor.read());
                break; 
            }     
            case RobotState::SLEEP: {
                
                if (sleep == 0) {
                    valve.setOpen(0.0f);
                    printf("going to sleep\n");
                    sleep = 1;
                }

                break; 
            }
            default: {
                break; // do nothing
            }      
        }


        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                valve.setOpen(0.0f);
                enable_motors = 0;                      //geht das (valve braucht motor)
                robot_state = INIT_EXECUTION;
                do_reset_all_once = false;

                // reset variables and objects
                user_led = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0) {
            printf("Warning: Main task took longer than main_task_period_ms\n");
            printf("main_task_period_ms: %d\n", main_task_period_ms);
        }
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}