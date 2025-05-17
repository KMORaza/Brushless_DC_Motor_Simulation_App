This software simulates a brushless DC motor and I wrote this software in C# programming language. This software is a robust tool for simulation of brushless DC motor yet it can be said that development of this software is still in progress because I have in mind some more features which I might integrate into it whenever my schedule allows. 
So I'll write enhanced version of this software when I'll get enough time but I ain't sure about when I'll get sufficient time. _(April 16 2025)_

*Update*:- Since I wanted to improve this software, I have incorporated some of the features. I have written [enhanced version](https://github.com/KMORaza/Brushless_DC_Motor_Simulation_App/tree/main/Brushless%20DC%20Motor%20Simulation%20App/BrushlessDCMotorSimulation) of this software. _(April 18 2025)_

---

## Brushless DC Motor Simulation Software

* Brushless DC Motor model :—
  * Computes phase currents based on applied voltages, back-EMF, phase resistance, and inductance (adjusted for magnetic saturation).
  * Supports fault conditions like phase loss (zero current in a phase) or short circuits (low resistance between phases).
  * Calculates back-EMF for each phase, using trapezoidal or sinusoidal waveforms depending on the motor type.
  * Transforms phase currents to d-q frame for Field-Oriented Control (FOC) using Park and Clarke transforms.
  * Models rotor motion using a second-order system (moment of inertia, friction, and torques).
  * Calculates electromagnetic torque (based on currents and torque constant), cogging torque (due to slot effects), ripple torque (harmonic effects), and friction torque.
  * Updates rotor angle and speed using a 4th-order Runge-Kutta method for numerical integration.
  * Provides speed in RPM and commutation sector for 6-step control.
  * Configurable motor type (trapezoidal or sinusoidal).
  * Fault injection for phase loss or short circuits.
  * Inductance variation due to magnetic saturation.
  * Realistic torque components (cogging, ripple) for accurate dynamics.
* Inverter model 
