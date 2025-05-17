This software simulates a brushless DC motor and I wrote this software in C# programming language. This software is a robust tool for simulation of brushless DC motor yet it can be said that development of this software is still in progress because I have in mind some more features which I might integrate into it whenever my schedule allows. 
So I'll write enhanced version of this software when I'll get enough time but I ain't sure about when I'll get sufficient time. _(April 16 2025)_

**Update**:- Because I wanted to improve this software, I have incorporated some more improvements. I have written the [enhanced version](https://github.com/KMORaza/Brushless_DC_Motor_Simulation_App/tree/main/Brushless%20DC%20Motor%20Simulation%20App/BrushlessDCMotorSimulation) of this software. _(April 18 2025)_

---

# Brushless DC Motor Simulation Software

### Brushless DC Motor Model
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
### Inverter Model
  * Simulates the power electronics driving the motor, converting DC link voltage to phase voltages.
  * Supports two switching modes: Ideal (no losses) and Realistic (includes switching and conduction losses, dead time, and slew effects).
  * Implements 6-step PWM (for trapezoidal control) or SVPWM (for sinusoidal FOC).
  * Converts desired voltages to duty cycles, applying constraints (e.g., maximum DC link voltage).
  * Calculates output voltages, accounting for MOSFET on-resistance and dead time distortion in realistic mode.
  * Tracks switching losses (based on MOSFET switching energies) and conduction losses.
  * Estimates inverter temperature based on power losses and supports thermal derating and shutdown when temperature exceeds thresholds.
  * Realistic switching effects (dead time, rise/fall times); Thermal monitoring for protection; Flexible PWM strategies (6-step or SVPWM).
### DC Power Supply Model
  * Updates DC link voltage based on capacitor charge and inverter input current (sum of absolute phase currents).
  * Models voltage drop due to ESR and recharge from a nominal voltage source.
  * Ensures non-negative voltage.
  * Supports overvoltage fault injection (e.g., setting nominal voltage to 32V, above the 30V threshold).
  * Dynamic voltage response to load changes; Fault injection for overvoltage testing; Reset functionality to restore nominal conditions.
### Motor Controller System
  * Trapezoidal Control (6-Step PWM)
    * Uses commutation sector (from rotor angle or fault-affected sensor) to energize two phases at a time.
    * Employs PID controllers for current regulation in active phases, with torque demand derived from speed error.
    * Applies voltages via the inverter in 6-step mode.
  * Sinusoidal Control (FOC)
    * PID Mode
      * Converts speed error to q-axis current reference using a speed PID controller.
      * Sets d-axis current reference to 0 for MTPA or adjusts for field weakening at high speeds.
      * Uses two PID controllers to regulate d- and q-axis currents, computing d-q voltages.
      * Applies inverse Park and Clarke transforms to convert d-q voltages to phase voltages.
    * MPC Mode
      * Delegates control to MPC controller model which computes optimal d- and q-axis voltages.
      * Transforms voltages to phase voltages for inverter application.
  * MTPA: Maximizes torque per ampere for surface-mount PMSMs.
  * Field Weakening: Reduces back-EMF at high speeds, respecting voltage and current limits.
  * Adaptive Control: Adjusts PID gains based on estimated resistance and inductance.
  * Integrates with fault manager to respect derating factors or stop operation during protection events.
### Model Predictive Control
  * Uses a discretized state-space model to predict motor behavior.
  * Defines cost function with weights for speed error, d- and q-axis current errors, and control effort.
  * Performs grid search, respecting voltage and current constraints.
  * Selects control inputs minimizing the cost over the prediction horizon (default N=5).
  * Accounts for load torque and back-EMF as disturbances.
  * Predictive control for improved performance; Constraint handling (voltage, current limits); Tunable parameters for flexibility.
### PID Controller Model
  * Computes control output using proportional, integral, and derivative terms.
  * Includes integral anti-windup with configurable limits.
  * Supports dynamic gain updates and setpoint changes.
  * Resets integral and error terms when stopping the motor.
  * Anti-windup for stable control.
  * Flexible gain tuning.
### Parameter Estimation 
  * Uses Recursive Least Squares (RLS) to estimate motor parameters (phase resistance and inductance).
  * Updates estimates based on phase voltages, currents, back-EMF, and current derivative.
  * Uses a forgetting factor (lambda = 0.99) to prioritize recent data.
  * Maintains a covariance matrix to adapt estimation gains.
  * Constrains estimated parameters to realistic bounds (0.1–10 Ω, 0.1–10 mH).
  * Skips estimation for low currents (|I| < 0.1 A) to avoid noise.
### Fault Management 
  * Fault Injection
    * Phase Loss: Sets a phase current to zero.
    * Short Circuit: Connects two phases with low resistance (0.01 Ω).
    * Sensor Failure: Simulates stuck or noisy rotor angle sensor.
    * Overvoltage: Increases DC link voltage (e.g., to 32V).
    * Overcurrent: Flags currents exceeding 10 A.
  * Monitors overcurrent (>10 A) and overvoltage (>30 V), triggering shutdowns.
  * Applies thermal derating when inverter temperature exceeds 80°C, shutting down at 120°C.
  * Provides derating factor (0–1) based on temperature.  
### Thermal Model
  * Estimates inverter temperature based on power losses.
  * Calculates temperature using a simplified model with thermal capacitance for inertia.
  * Includes natural cooling toward ambient temperature (25°C).
  * Ensures temperature remains above ambient.
  * Supports derating and shutdown logic.
### Simulation Engine
  * Integrates all components, updating the system state at each time step.
  * Executes simulation in 100 µs sub-steps (100 sub-steps per 10 ms) for numerical stability.
  * Handles exceptions by stopping the motor and reporting errors.
  * Sets load torque (0–0.1 Nm) and speed reference (0–3000 RPM).
  * Centralized system integration.
  
---

| ![](https://github.com/KMORaza/Brushless_DC_Motor_Simulation_App/blob/main/Brushless%20DC%20Motor%20Simulation%20App/BrushlessDCMotorSimulation/screenshots/screen%20(1).png) | ![](https://github.com/KMORaza/Brushless_DC_Motor_Simulation_App/blob/main/Brushless%20DC%20Motor%20Simulation%20App/BrushlessDCMotorSimulation/screenshots/screen%20(2).png) |
|------|------|
| ![](https://github.com/KMORaza/Brushless_DC_Motor_Simulation_App/blob/main/Brushless%20DC%20Motor%20Simulation%20App/BrushlessDCMotorSimulation/screenshots/screen%20(3).png) | ![](https://github.com/KMORaza/Brushless_DC_Motor_Simulation_App/blob/main/Brushless%20DC%20Motor%20Simulation%20App/BrushlessDCMotorSimulation/screenshots/screen%20(4).png) |







