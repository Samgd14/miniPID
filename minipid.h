class pid{
    public:
        // Initializes the class with the constants
        pid(double kp_t, double ki_t, double kd_t){
            kp = kp_t;
            ki = ki_t;
            kd = kd_t;
            vLimiter = false;
            maxOutput = 1;
            minOutput = -1;
        }

        // Initializes the class with the constants and position limits, and enables it
        pid(double kp_t, double ki_t, double kd_t, double maxOutput_t, double minOutput_t){
            kp = kp_t;
            ki = ki_t;
            kd = kd_t;
            vLimiter = true;
            maxOutput = maxOutput_t;
            minOutput = minOutput_t;
        }

        // Used to update constants after initialization
        void values(double kp_t, double ki_t, double kd_t){
            kp = kp_t;
            ki = ki_t;
            kd = kd_t;
        }

        // Used to change individual constants after initialization
        void setKp(double kp_t){
            kp = kp_t;
        }
        void setKi(double ki_t){
            ki = ki_t;
        }
        void setKd(double kd_t){
            kd = kd_t;
        }

        //Returns the value limiter's current state
        bool pLimit(){
            return vLimiter;
        }

        //Edits and returns the value limiter's current state
        bool pLimit(bool vLimiter_t){
            vLimiter = vLimiter_t;

            return vLimiter;
        }

        //Edits the value limiter's values and enables it
        void pLimit(double maxOutput_t, double minOutput_t){
            vLimiter = true;
            maxOutput = maxOutput_t;
            minOutput = minOutput_t;
        }

        // Updates the PID's output calculation
        bool update(double input, double target, double delay){
            
            deltaT = delay / 1000;
            error = target - input; //Calculates the error
            
            integral = integral + (error * deltaT);
            derivative = (error / deltaT) - (lastError / lastDeltaT);
            
            out = (kp * error) + (ki * integral) + (kd * derivative);

            rawOut = out;

            lastDeltaT = deltaT;
            lastError = error;

            if(vLimiter){
                if(out > maxOutput){
                    out = maxOutput;
                }
                else if(out < minOutput){
                    out = minOutput;
                }
            }

            return true;
        }

        // Returns the current output
        double output(){
            return out;
        }

        //Returns if the PID has reached it's desired value
        bool isDone(){
            if(error == 0){
                return true;
            }
            return false;
        }

        //Returns if the PID has reached it's desired value in a limited range
        bool isDone(double range){
            if((error < range && error > 0) || (-error < range && error < 0) || (error == 0)){
                return true;
            }
            return false;
        }

        //Returns the last calculated error
        double getError(){
            return error;
        }

        //Resets every accumulated/output value to 0
        void reset(){
            out = 0;
            rawOut = 0;

            error = 0;
            lastError = 0;

            deltaT = 0;
            lastDeltaT = 0;

            integral = 0;
            derivative = 0;
        }

    private:
        double kp, ki, kd; // Multipliers for the proportionnal, integral and differential factors
        double out, rawOut; // Output variable, contains the last calculated output
        double maxOutput, minOutput; // Maximum/minimum value for the output

        double error, lastError; // Calculated errors for the PID calculation
        double deltaT, lastDeltaT; // Time deltas for calculating derivatives and integrals
        double integral, derivative; // Parameters for integrals and derivatives

        bool vLimiter, rLimiter;
};