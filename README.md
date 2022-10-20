miniPID
======
Simple and light library to calculate a PID!

Initialization
------
```c++
pid(double kp, double ki, double kd)
```
Creates an object using the specified PID parameters and with the limited output set to the default values (-1 to 1).

---

```c++
pid(double kp, double ki, double kd, double maxOutput, double minOutput)
```
Creates an object using the specified PID parameters and with the `limitedOutput` clamped to the specified values.

Functions
------

```c++
void values(double kp, double ki, double kd)
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
void setLimits(double maxOutput_t, double minOutput_t)
```
Sets the minimum and maximum values to which `limitedOutput` is clamped.

---

```c++
bool update(double input, double target, double delay)
```
Updates the PID controller\'s calculation. `input` is the input value for the controller, and `target` is the target the PID is trying to achieve. Both of these values are used to calculate the error that is used for computing the output.
`delay` is the time in ms between each update and can be used to compensate for irregular timing in the controller.
Still, the `update` function should be updated as regularly as possible.

---

```c++
double output()
```
Returns the output last computed by the `update` function.

---

```c++
double limitedOutput()
```
Returns the output last computed by the `update` function, but clamped within `minOutput` and `maxOutput`.

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
