namespace BrushlessDCMotorSimulation
{
    public static class SimulationConstants
    {
        public const double SupplyVoltage = 24.0; // Volts
        public const double PhaseResistance = 1.0; // Ohms
        public const double MutualInductance = 0.001; // Henries
        public const double TorqueConstant = 0.1; // Nm/A
        public const double BackEmfConstant = 0.1; // V/(rad/s)
        public const double MomentOfInertia = 0.001; // kg·m²
        public const double FrictionCoefficient = 0.0001; // Nm/(rad/s)
        public const double DefaultLoadTorque = 0.02; // Nm
        public const double DefaultKp = 0.5; // Speed PID
        public const double DefaultKi = 0.1;
        public const double DefaultKd = 0.01;
        public const double DefaultCurrentKp = 2.0; // Current PI
        public const double DefaultCurrentKi = 1.0;

        // Inverter and MOSFET parameters
        public const double MosfetOnResistance = 0.05; // Ohms
        public const double MosfetRiseTime = 100e-9; // Seconds (100 ns)
        public const double MosfetFallTime = 100e-9; // Seconds (100 ns)
        public const double MosfetSwitchingEnergyOn = 50e-6; // Joules (50 µJ)
        public const double MosfetSwitchingEnergyOff = 30e-6; // Joules (30 µJ)
        public const double DeadTime = 1e-6; // Seconds (1 µs)
        public const double SwitchingFrequency = 20e3; // Hz (20 kHz)

        // DC Link parameters
        public const double CapacitorValue = 470e-6; // Farads (470 µF)
        public const double CapacitorESR = 0.05; // Ohms (50 mΩ)

        // Fault thresholds
        public const double MaxCurrent = 10.0; // Amps
        public const double MaxVoltage = 30.0; // Volts
        public const double ShortCircuitResistance = 0.01; // Ohms

        // Thermal parameters
        public const double ThermalResistance = 10.0; // °C/W
        public const double AmbientTemperature = 25.0; // °C
        public const double DeratingStartTemp = 80.0; // °C
        public const double MaxTemperature = 120.0; // °C
    }
}