using System;

namespace BrushlessDCMotorSimulation
{
    public class ThermalModel
    {
        public double Temperature { get; private set; }
        private readonly double thermalResistance;
        private readonly double ambientTemperature;

        public ThermalModel()
        {
            thermalResistance = SimulationConstants.ThermalResistance;
            ambientTemperature = SimulationConstants.AmbientTemperature;
            Temperature = ambientTemperature;
        }

        public void Update(double powerLoss, double deltaTime)
        {
            // Simplified thermal model: T = T_amb + P * R_th
            // Add some inertia to simulate thermal capacitance
            double deltaTemp = powerLoss * thermalResistance * deltaTime;
            Temperature += deltaTemp;
            // Natural cooling
            double cooling = (Temperature - ambientTemperature) * 0.1 * deltaTime;
            Temperature -= cooling;
            Temperature = Math.Max(ambientTemperature, Temperature);
        }

        public void Reset()
        {
            Temperature = ambientTemperature;
        }
    }
}