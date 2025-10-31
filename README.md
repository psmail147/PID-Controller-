# Closed-Loop PID Control System in Python

This project implements and tests a **closed-loop PID control system** in Python to guide a Pioneer mobile robot through wall-following behaviour in **CoppeliaSim**.  
It provides both a **classical PID** and a **fractional-order PID (FOPID)** controller, highlighting the mathematical and algorithmic principles behind dynamic control systems.

---

## Theoretical Background

The control law of a standard PID controller is governed by the **differential equation**:

<p align="center">
  <b>u(t) = K<sub>p</sub> e(t) + K<sub>i</sub> ∫₀ᵗ e(τ) dτ + K<sub>d</sub> (de(t)/dt)</b>
</p>

where  
- **u(t)** is the control signal,  
- **e(t)** is the instantaneous error,  
- **K<sub>p</sub>**, **K<sub>i</sub>**, and **K<sub>d</sub>** are the proportional, integral, and derivative gains.  

This continuous-time control law is discretised in the implementation for simulation in CoppeliaSim, allowing the controller to operate on sampled sensor data in real time.  

The **fractional-order PID (FOPID)** extends the standard model using **fractional calculus**, replacing integer-order differentiation and integration with real-valued orders λ and μ:

<p align="center">
  <b>u(t) = K<sub>p</sub> e(t) + K<sub>i</sub> D<sup>-λ</sup> e(t) + K<sub>d</sub> D<sup>μ</sup> e(t)</b>
</p>

These fractional operators are approximated numerically using the **Grünwald–Letnikov method**, implemented in `pid_controller.py`.


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
- Tunable parameters for K<sub>p</sub>, K<sub>i</sub>, K<sub>d</sub>, λ, and μ
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
