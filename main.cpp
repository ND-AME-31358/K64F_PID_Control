#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"

// Define number of communication parameters with matlab
#define NUM_INPUTS 6
#define NUM_OUTPUTS 5

Serial pc(USBTX, USBRX,115200);     // USB Serial Terminal for debugging
ExperimentServer server;            // Object that lets us communicate with MATLAB
Timer t;                            // Timer to measure elapsed time of experiment
Ticker currentLoopTicker;           // Ticker to call high frequency current loop

// Assign digital/analog pins for control and sensing
PwmOut     M1PWM(D10);              // Motor PWM output
DigitalOut M1INA(D2);               // Motor forward enable
DigitalOut M1INB(D3);               // Motor backward enable
AnalogIn   CS(A2);                  // Current sensor

float err_integration = 0.0;

// Create a quadrature encoder
// 64(counts/motor rev)*18.75(gear ratio) = 1200(counts/rev)
// Pins A, B, no index, 1200 counts/rev, Quadrature encoding
QEI encoder(D3,D5, NC, 1200 , QEI::X4_ENCODING);
const float degPerTick = 360.0/1200.0;

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM);

const float SupplyVoltage = 12;     // Supply voltage in Volts
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM);

int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();

    // PWM period should nominally be a multiple of our control loop
    M1PWM.period_us(50);
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) {
            float angle_des   = input_params[0]; // Desired angle
            float vel_des     = input_params[1]; // Desired velocity
            float Kp   = input_params[2]; // Kp
            float Kd   = input_params[3]; // Kd
            float Ki   = input_params[4]; // Ki
            float ExpTime = input_params[5]; // Expriement time in second

            // Setup experiment
            t.reset();
            t.start();
            encoder.reset();
            setMotorVoltage(0,M1INA,M1INB,M1PWM);
            err_integration = 0.0; // Reset error

            // Run experiment
            while( t.read() < ExpTime ) { 
                
                // Read angle from encoder
                float angle = (float)encoder.getPulses()*degPerTick;
                // Read velocity from encoder
                float velocity = encoder.getVelocity()*degPerTick;
                // Read the current sensor value
                float current = 36.7f * CS - 18.3f;
                // Integrate the error
                err_integration += angle - angle_des;
                float voltage;
                voltage = Kp * (angle - angle_des) + Kd * (velocity - vel_des) + Ki * err_integration;
                setMotorVoltage(voltage,M1INA,M1INB,M1PWM);
                /**********TODO: Student task************/

                // Form output to send to MATLAB    
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = angle;
                output_data[2] = velocity;
                output_data[3] = voltage;
                output_data[4] = current;
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                wait(.001);                  // Control and sending data in 1kHz
            }     
            // Cleanup after experiment
            server.setExperimentComplete();
            setMotorVoltage(0,M1INA,M1INB,M1PWM);
        } // end if
    } // end while
} // end main



//Set motor voltage (nagetive means reverse)
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM){
    setMotorDuty(voltage * SupplyVoltage, INA, INB, PWM);
}

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, PwmOut &PWM)
{
    unsigned char reverse = 0;

    if (duty < 0) {
        duty = -duty;  // Make duty a positive quantity
        reverse = 1;  // Preserve the direction
    }

    if (duty == 0) {
        INA = 0;  // Make the motor coast no
        INB = 0;  // matter which direction it is spinning.
    } else if (reverse) {
        INA = 0;
        INB = 1;
    } else {
        INA = 1;
        INB = 0;
    }

    PWM.write(duty);
}