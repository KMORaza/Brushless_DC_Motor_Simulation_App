using System;

namespace BrushlessDCMotorSimulation
{
    public class DCLink
    {
        public double Voltage { get; private set; }
        private double charge; // Coulombs
        private readonly double capacitance;
        private readonly double esr;
        private double nominalVoltage;
        private bool overvoltageInjected;

        public DCLink()
        {
            capacitance = SimulationConstants.CapacitorValue;
            esr = SimulationConstants.CapacitorESR;
            nominalVoltage = SimulationConstants.SupplyVoltage;
            Voltage = nominalVoltage;
            charge = capacitance * Voltage;
            overvoltageInjected = false;
        }

        public void InjectOvervoltage()
        {
            nominalVoltage = 32.0; // Above MaxVoltage threshold
            overvoltageInjected = true;
        }

        public void ClearOvervoltage()
        {
            nominalVoltage = SimulationConstants.SupplyVoltage;
            overvoltageInjected = false;
        }

        public bool IsOvervoltageInjected()
        {
            return overvoltageInjected;
        }

        public void Update(double[] phaseCurrents, double deltaTime)
        {
            // Calculate inverter input current
            double i_inv = Math.Abs(phaseCurrents[0]) + Math.Abs(phaseCurrents[1]) + Math.Abs(phaseCurrents[2]);

            // Update charge: dQ/dt = -I_inv
            charge -= i_inv * deltaTime;

            // Calculate new voltage: V = Q/C
            Voltage = charge / capacitance;

            // Include ESR voltage drop
            Voltage -= i_inv * esr;

            // Ensure voltage doesn't go negative
            Voltage = Math.Max(0, Voltage);

            // Recharge model
            double rechargeCurrent = (nominalVoltage - Voltage) / esr;
            charge += rechargeCurrent * deltaTime;
            Voltage = charge / capacitance;
        }

        public void Reset()
        {
            nominalVoltage = SimulationConstants.SupplyVoltage;
            Voltage = nominalVoltage;
            charge = capacitance * Voltage;
            overvoltageInjected = false;
        }
    }
}