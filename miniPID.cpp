#include "miniPID.h"

// Initializes the class with the constants
PID::PID(double kp_, double ki_, double kd_){
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

// Initializes the class with the constants and position limits, and enables it
PID::PID(double kp_t, double ki_t, double kd_t, double maxValue_t, double minValue_t){
    kp = kp_t;
    ki = ki_t;
    kd = kd_t;
    vLimiter = true;
    maxValue = maxValue_t;
    minValue = minValue_t;
}

PID::PID(double kp_, double ki_, double kd_, double maxValue_, double minValue_, double maxRate_, double minRate_){
    kp = kp_;
    ki = ki_;
    kd = kd_;
    vLimiter = true;
    maxValue = maxValue_;
    minValue = minValue_;
    rLimiter = true;
    maxRate = maxRate_;
    minRate = minRate_;
}

// Updates the PID's output calculation
bool PID::update(double input, double target, double delay){
    
    deltaT = delay / 1000;
    error = target - input; //Calculates the error
    
    integral = integral + (error * deltaT);
    derivative = (error / deltaT) - (lastError / lastDeltaT);
    
    out = (kp * error) + (ki * integral) + (kd * derivative);

    rawOut = out;

    //If enabled, caps the output value
    if(vLimiter){
        if(out > maxValue){
            out = maxValue;
        }
        else if(out < minValue){
            out = minValue;
        }
    }  

    //If enabled, caps the rate of change of the output
    if(rLimiter){
        currentRate = (out - lastOut) / deltaT;

        if(currentRate > maxRate){
            out = lastOut + maxRate * deltaT;
        }
        else if(currentRate < minRate){
            out = lastOut + minRate * deltaT;
        }
    }

    lastOut = out;
    lastDeltaT = deltaT;
    lastError = error;

    return true;
}

//Resets every accumulated/output value to 0
void PID::reset(){
    out = 0;
    rawOut = 0;
    lastOut = 0;

    currentRate = 0;

    error = 0;
    lastError = 0;

    deltaT = 0;
    lastDeltaT = 0;

    integral = 0;
    derivative = 0;
}