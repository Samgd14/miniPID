miniPID
======
Simple and light library to calculate a PID!
I'm currently working on this version, so use the previous release if you use it in projects.
This new version will break compatibility on the value limiter and introduce new rate and acceleration limiters.
The new version of the value limiter and the rate limiter are now implemented.

Constructors
------
```c++
PID(double kp, double ki, double kd)
```
Creates an object using the specified PID parameters and with the limited output set to the default values (-1 to 1).

---

```c++
PID(double kp, double ki, double kd, double maxValue, double minValue)
```
Creates an object using the specified PID parameters and with the value limiter enabled on the output.

---

```c++
PID(double kp, double ki, double kd, double maxValue, double minValue, double maxRate, double minRate)
```
Creates an object using the specified PID parameters and with the value limiter enabled on the output.

Functions
------

```c++
void setKs(double kp, double ki, double kd)
```
Sets the `ki`, `kp`, and `kd` values after initialization.

---

```c++
void setKp(double kp)
```
```c++
void setKi(double ki)
```
```c++
void setKd(double kd)
```
Individually sets the PID constants for the controller.

---

```c++
bool getVLimitState()
```
Get the current state of the value limiter.

---

```c++
bool enableVLimit(bool vLimiter)
```
Enables/disables the value limiter and returns it's state.

---

```c++
void setVLimit(double maxValue, double minValue)
```
Changes the values to which the output is clamped to.


---

```c++
bool getRLimitState()
```
Get the current state of the rate limiter.

---

```c++
bool enableRLimit(bool rLimiter)
```
Enables/disables the rate limiter and returns it's state.

---

```c++
void setRLimit(double maxRate, double minRate)
```
Sets the rates of change to which the output is clamped to.

---

```c++
bool update(double input, double target, double delay)
```
Updates the PID controller\'s calculation. `input` is the input value for the controller, and `target` is the target the PID is trying to achieve. Both of these values are used to calculate the error that is used for computing the output, which is clamped by the value/rate limiters, if they are enabled.
`delay` is the time in ms between each update and can be used to compensate for irregular timing in the controller.
Still, the `update` function should be updated as regularly as possible.

---

```c++
double getOutput()
```
Returns the output last computed by the `update` function.

---

```c++
double getRawOutput()
```
Returns the output last computed by the `update` function without the value/rate limiters.

---

```c++
bool isDone()
```
```c++
bool isDone(double range)
```
Returns true if the last calculated error is equal to zero.
If a range is specified, it will return true if the last calculated error is in that range.

---

```c++
double getError()
```
Returns the last calculated error.

---

```c++
void reset()
```
Resets every accumulated/output value to 0.