# Closed-Loop PID Control System in Python

This project implements and tests a **closed-loop PID control system** in Python to guide a Pioneer mobile robot through wall-following behaviour in **CoppeliaSim**.  
It provides both a **classical PID** and a **fractional-order PID (FOPID)** controller, highlighting the mathematical and algorithmic principles behind dynamic control systems.

---

## 🧠 Theoretical Background

The control law of a standard PID controller is governed by the **differential equation**:

\[
u(t) = K_p e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}
\]

where  
- \(u(t)\) is the control signal,  
- \(e(t)\) is the instantaneous error,  
- \(K_p, K_i, K_d\) are the proportional, integral, and derivative gains.  

This project discretises the above equation for real-time control within a simulation environment. The controller continuously updates the control output based on sampled error values and elapsed time between iterations.

The **fractional-order PID (FOPID)** extends this principle using **fractional calculus**, replacing integer-order integration and differentiation with non-integer orders \( \lambda \) and \( \mu \):

\[
u(t) = K_p e(t) + K_i D_t^{-\lambda} e(t) + K_d D_t^{\mu} e(t)
\]

These fractional derivatives are approximated numerically using the **Grünwald–Letnikov** method, implemented in the file `pid_controller.py`.

---

## Project Structure

│
├── main.py # Entry point for running the simulation and control loop
├── pid_controller.py # PID and fractional-order PID implementations
├── optimizer.py # Script to tune controller gains using stochastic optimisation
├── robot.py # Robot model and sensor interface for wall-following
├── PID_Control_Optimize.sln / .pyproj # Project solution files for Visual Studio
├── .gitignore, .gitattributes # Git configuration


- **`main.py`** launches the simulation, connects to the robot in CoppeliaSim, and applies control commands.  
- **`pid_controller.py`** defines two controller classes: `PID_Controller` and `PID_Controller_Fract`. The latter computes fractional derivatives and integrals using discrete convolution.  
- **`optimizer.py`** provides functions to automatically tune the PID parameters using stochastic or evolutionary search.  
- **`robot.py`** contains sensor reading, motor commands, and wall-following logic.

---

## Features

- Classical and fractional-order PID controllers  
- Discrete-time implementation of continuous differential equations  
- Tunable parameters for \(K_p\), \(K_i\), \(K_d\), \(λ\), and \(μ\)  
- Modular structure for easy integration with robotic simulation environments  
- Example configuration for **CoppeliaSim Pioneer P3-DX** robot

---

## Usage

1. Start **CoppeliaSim** and load the Pioneer robot scene.  
2. Run `main.py` to launch the closed-loop control.  
3. Modify parameters in `pid_controller.py` or use `optimizer.py` to tune automatically.

---

##  Dependencies

- Python 3.8+
- `math`
- `collections`
- `CoppeliaSim Remote API` (for communication with the simulator)

---

*Author: Philip Smith*  
*Email: psmail147@gmail.com*  
*GitHub: [psmail147](https://github.com/psmail147)*
