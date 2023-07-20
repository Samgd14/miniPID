#ifndef minipid_h
#define minipid_h

class miniPID{
    public:

    /*
    Class constructors
    */

    //Initializes the class alone
    miniPID(){};

    // Initializes the class with the constants
    miniPID(double kp_, double ki_, double kd_);

    // Initializes the class with the constants and position limits, and enables it
    miniPID(double kp_, double ki_, double kd_, double maxValue_, double minValue_);

    // Initializes the class with the constants and position limits, and enables it
    miniPID(double kp_, double ki_, double kd_, double maxValue_, double minValue_, double maxRate_, double minRate_);

    // Used to update constants after initialization
    void setKs(double kp_, double ki_, double kd_)
    {kp = kp_; ki = ki_; kd = kd_;}

    // Used to change individual constants after initialization
    void setKp(double kp_)
    {kp = kp_;}

    void setKi(double ki_)
    {ki = ki_;}

    void setKd(double kd_)
    {kd = kd_;}

    /*
    Value limiter
    */

    //Returns the value limiter's current state
    bool getVLimitState() const
    {return vLimiter;}

    //Edits and returns the value limiter's current state
    bool enableVLimit(bool vLimiter_){
        vLimiter = vLimiter_;
        return vLimiter;
    }

    //Sets the value limiter's limits
    void setVLimit(double maxValue_, double minValue_){
        maxValue = maxValue_;
        minValue = minValue_;
    }

    /*
    Rate limiter
    */

    //Returns the rate limiter's current state
    bool getRLimitState() const
    {return rLimiter;}

    //Edits and returns the rate limiter's current state
    bool enableRLimit(bool rLimiter_){
        rLimiter = rLimiter_;
        return rLimiter;
    }

    //Sets the rate limiter's limits
    void setRLimit(double maxRate_, double minRate_){
        maxRate = maxRate_;
        minRate = minRate_;
    }

    /*
    Output gets and sets
    */

    // Updates the PID's output calculation
    bool update(double input, double target, double delay);

    // Returns the current output
    double getOutput() const
    {return out;}

    //Returns the current output before limiters
    double getRawOutput() const
    {return rawOut;}

    //Returns true if the PID has reached it's desired value
    bool isDone() const {
        if(error == 0){
            return true;
        }
        return false;
    }

    //Returns if the PID has reached it's desired value in a limited range
    bool isDone(double range) const {
        if((error < range && error > 0) || (-error < range && error < 0) || (error == 0)){
            return true;
        }
        return false;
    }

    //Returns the last calculated error
    double getError() const
    {return error;}

    //Resets every accumulated/output value to 0
    void reset();

    private:

    double kp = 0, ki = 0, kd = 0; // Multipliers for the proportionnal, integral and differential factors
    double out, rawOut, lastOut; // Output variable, contains the last calculated output
    
    double error, lastError; // Calculated errors for the PID calculation
    double deltaT, lastDeltaT; // Time deltas for calculating derivatives and integrals
    double integral, derivative; // Parameters for integrals and derivatives

    double maxValue = 0, minValue = 0; // Maximum/minimum value for the output
    double maxRate = 0, minRate = 0; //Maximum/minimum rate change values
    double currentRate; //Current calculated rate for the rate limiter
    bool vLimiter = false, rLimiter = false; //Limiter states
};
#endif