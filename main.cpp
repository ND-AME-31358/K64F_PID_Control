#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "MovingAverageFilter.h"

#define NUM_INPUTS 5
#define NUM_OUTPUTS 5

struct MovingAverageFilter<float, 1> MAF(0.0);

const float degPerTick = 360.0/1200.0;

Serial pc(USBTX, USBRX,115200);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
PwmOut motorPWM(D10);        // Motor PWM output
DigitalOut motorFwd(D7);    // Motor forward enable
DigitalOut motorRev(D8);    // Motor backward enable
AnalogIn   CS(A3);
Timer t;                    // Timer to measure elapsed time of experiment
Ticker currentLoopTicker;
float err_inter = 0;

QEI encoder(D14,D15, NC, 1200 , QEI::X4_ENCODING); // Pins D3, D4, no index, 1200 counts/rev, Quadrature encoding

void setMotorVoltage(float speed, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM);

int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();

    // PWM period should nominally be a multiple of our control loop
    motorPWM.period_us(50);
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) {
            float angle_des   = input_params[0]; // Desired angle
            float Kp   = input_params[1]; // Kp
            float Kd   = input_params[2]; // Kd
            float Ki   = input_params[3]; // Ki
            float ExpTime = input_params[4]; // Expriement time in second

            err_inter = 0;
        
            // Setup experiment
            t.reset();
            t.start();
            encoder.reset();
            setMotorVoltage(0,motorFwd,motorRev,motorPWM);

            // Run experiment
            // currentLoopTicker.attach(&currentLoop,0.0002);
            while( t.read() < ExpTime ) { 
                
                float angle = (float)encoder.getPulses()*degPerTick;
                float velocity = MAF.update(encoder.getVelocity()*degPerTick);
                float current = 36.7f * CS - 18.3f;
                err_inter += angle - angle_des;
                float duty;
                duty = Kp * (angle - angle_des) + Kd * (velocity - 0.0) + Ki * err_inter;
                setMotorVoltage(duty,motorFwd,motorRev,motorPWM);

                // Form output to send to MATLAB    
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = angle;
                output_data[2] = velocity;
                output_data[3] = duty;
                output_data[4] = current;
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                wait(.001); 
            }     
            // Cleanup after experiment
            server.setExperimentComplete();
            setMotorVoltage(0,motorFwd,motorRev,motorPWM);
        } // end if
    } // end while
} // end main



// Set motor 1 voltage [-1.0f, 1.0f]
void setMotorVoltage(float speed, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM)
{
    unsigned char reverse = 0;

    if (speed < 0) {
        speed = -speed;  // Make speed a positive quantity
        reverse = 1;  // Preserve the direction
    }

    PWM.write(speed);

    if (speed == 0) {
        INA = 0;  // Make the motor coast no
        INB = 0;  // matter which direction it is spinning.
    } else if (reverse) {
        INA = 0;
        INB = 1;
    } else {
        INA = 1;
        INB = 0;
    }
}