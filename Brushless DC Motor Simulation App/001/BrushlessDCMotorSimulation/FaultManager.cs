using System;

namespace BrushlessDCMotorSimulation
{
    public class FaultManager
    {
        private readonly BLDCMotor motor;
        private readonly Inverter inverter;
        private readonly DCLink dcLink;
        private bool isPhaseLossActive;
        private bool isShortCircuitActive;
        private bool isSensorFailureActive;
        private bool isOvervoltageActive;
        private bool isOvercurrentActive;
        private bool isOvercurrentShutdown;
        private bool isOvervoltageShutdown;
        private double lastSensorAngle;
        private Random random;

        public FaultManager(BLDCMotor motor, Inverter inverter, DCLink dcLink)
        {
            this.motor = motor ?? throw new ArgumentNullException(nameof(motor));
            this.inverter = inverter ?? throw new ArgumentNullException(nameof(inverter));
            this.dcLink = dcLink ?? throw new ArgumentNullException(nameof(dcLink));
            isPhaseLossActive = false;
            isShortCircuitActive = false;
            isSensorFailureActive = false;
            isOvervoltageActive = false;
            isOvercurrentActive = false;
            isOvercurrentShutdown = false;
            isOvervoltageShutdown = false;
            lastSensorAngle = 0;
            random = new Random();
        }

        public void InjectPhaseLoss(int phase)
        {
            motor.SetPhaseLoss(phase);
            isPhaseLossActive = true;
        }

        public void InjectShortCircuit(int phase1, int phase2)
        {
            motor.SetShortCircuit(phase1, phase2);
            isShortCircuitActive = true;
        }

        public void InjectSensorFailure()
        {
            isSensorFailureActive = true;
        }

        public void InjectOvervoltage()
        {
            dcLink.InjectOvervoltage();
            isOvervoltageActive = true;
        }

        public void InjectOvercurrent()
        {
            isOvercurrentActive = true;
        }

        public void ClearFaults()
        {
            motor.ClearFaults();
            dcLink.ClearOvervoltage();
            isPhaseLossActive = false;
            isShortCircuitActive = false;
            isSensorFailureActive = false;
            isOvervoltageActive = false;
            isOvercurrentActive = false;
            isOvercurrentShutdown = false;
            isOvervoltageShutdown = false;
        }

        public bool IsPhaseLossActive => isPhaseLossActive;
        public bool IsShortCircuitActive => isShortCircuitActive;
        public bool IsSensorFailureActive => isSensorFailureActive;
        public bool IsOvervoltageActive => isOvervoltageActive;
        public bool IsOvercurrentActive => isOvercurrentActive;

        public bool IsProtectionActive()
        {
            return isOvercurrentShutdown || isOvervoltageShutdown || inverter.GetTemperature() >= SimulationConstants.MaxTemperature;
        }

        public double GetDeratingFactor()
        {
            double temp = inverter.GetTemperature();
            if (temp < SimulationConstants.DeratingStartTemp)
                return 1.0;
            if (temp >= SimulationConstants.MaxTemperature)
                return 0.0;
            return 1.0 - (temp - SimulationConstants.DeratingStartTemp) / (SimulationConstants.MaxTemperature - SimulationConstants.DeratingStartTemp);
        }

        public int GetEffectiveSector(int actualSector)
        {
            if (!isSensorFailureActive)
                return actualSector;
            // Simulate stuck or noisy sensor
            if (random.NextDouble() < 0.5)
                return (int)(lastSensorAngle % 360.0 / 60.0);
            lastSensorAngle = random.NextDouble() * 360.0;
            return (int)(lastSensorAngle % 360.0 / 60.0);
        }

        public void Update(double[] phaseCurrents, double dcLinkVoltage, double deltaTime)
        {
            // Check for overcurrent
            foreach (var current in phaseCurrents)
            {
                if (Math.Abs(current) > SimulationConstants.MaxCurrent)
                {
                    isOvercurrentActive = true;
                    isOvercurrentShutdown = true;
                }
            }

            // Check for overvoltage
            if (dcLinkVoltage > SimulationConstants.MaxVoltage)
            {
                isOvervoltageActive = true;
                isOvervoltageShutdown = true;
            }
        }

        public string GetProtectionStatus()
        {
            if (isOvercurrentShutdown)
                return "Overcurrent Shutdown";
            if (isOvervoltageShutdown)
                return "Overvoltage Shutdown";
            if (inverter.GetTemperature() >= SimulationConstants.MaxTemperature)
                return "Thermal Shutdown";
            if (inverter.GetTemperature() > SimulationConstants.DeratingStartTemp)
                return $"Thermal Derating ({inverter.GetTemperature():F1}°C)";
            return "Normal";
        }
    }
}