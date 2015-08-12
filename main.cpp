#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"

#define NUM_INPUTS 2
#define NUM_OUTPUTS 3

Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
PwmOut motorPMW(D5);        // Motor PMW output
DigitalOut motorFwd(D6);    // Motor forward enable
DigitalOut motorRev(D7);    // Motor backward enable
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoder(D3,D4, NC, 1200 , QEI::X4_ENCODING); // Pins D3, D4, no index, 1200 counts/rev, Quadrature encoding

int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();

    // PMW period should nominally be a multiple of our control loop
    motorPMW.period_us(500);
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) {
            float v1   = input_params[0]; // Voltage for first second
            float v2   = input_params[1]; // Voltage for second second
        
            // Setup experiment
            t.reset();
            t.start();
            encoder.reset();
            motorFwd = 1;
            motorRev = 0;
            motorPMW.write(0);

            // Run experiment
            while( t.read() < 2 ) { 
                // Perform control loop logic
                if (t.read() < 1) 
                    motorPMW.write(v1);
                else 
                    motorPMW.write(v2);
                    
                // Form output to send to MATLAB    
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = encoder.getPulses();
                output_data[2] = encoder.getVelocity();
             
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                wait(.001); 
            }     
            // Cleanup after experiment
            server.setExperimentComplete();
            motorPMW.write(0);
        } // end if
    } // end while
} // end main