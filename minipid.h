class pid{
    public:
        // Initializes the class with the constants
        pid(double kp_t, double ki_t, double kd_t){
            kp = kp_t;
            ki = ki_t;
            kd = kd_t;
            maxOutput = 1;
            minOutput = -1;
        }

        // Initializes the class with the constants and output llimits
        pid(double kp_t, double ki_t, double kd_t, double maxOutput_t, double minOutput_t){
            kp = kp_t;
            ki = ki_t;
            kd = kd_t;
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

        //Used to change output limits after initialization
        void setLimits(double maxOutput_t, double minOutput_t){
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

            if(out > maxOutput){
                limitedOut = maxOutput;
            }
            else if(out < minOutput){
                limitedOut = minOutput;
            }
            else{
                limitedOut = out;
            }

            lastDeltaT = deltaT;
            lastError = error;

            return true;
        }

        // Returns the current output
        double output(){
            return out;
        }

        // Returns the current limited output
        double limitedOutput(){
            return limitedOut;
        }

        //Returns if the PID has reached it's desired value in a limited range
        bool isDone(double range){
            if((error < range && error > 0) || (-error < range && error < 0) || (error == 0)){
                return true;
            }
            return false;
        }

        //Returns if the PID has reached it's desired value
        bool isDone(){
            if(error == 0){
                return true;
            }
            return false;
        }

    private:
        double kp, ki, kd; // Multipliers for the proportionnal, integral and differential factors
        double out, limitedOut; // Output variable, contains the last calculated output
        double maxOutput, minOutput; // Maximum/minimum value for the output

        double error, lastError; // Calculated errors for the PID calculation
        double deltaT, lastDeltaT; // Time deltas for calculating derivatives and integrals
        double integral, derivative; // Parameters for integrals and derivatives
};