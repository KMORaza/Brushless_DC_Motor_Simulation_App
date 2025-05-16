using System;

namespace BrushlessDCMotorSimulation
{
    public enum PwmMode { SixStep, SVPWM }

    public class ConcurrentModificationException : Exception
    {
        public ConcurrentModificationException(string message) : base(message) { }
    }

    public class MotorController
    {
        private readonly BLDCMotor motor;
        private readonly Inverter inverter;
        private readonly PIDController speedPid;
        private readonly PIDController[] currentPids;
        private readonly FaultManager faultManager;
        private double iqReference;
        private bool isRunning;
        private MotorType motorType;
        private PwmMode pwmMode;
        private bool useMtpa;
        private bool useFieldWeakening;

        public MotorController(BLDCMotor motor, Inverter inverter, FaultManager faultManager)
        {
            this.motor = motor ?? throw new ArgumentNullException(nameof(motor));
            this.inverter = inverter ?? throw new ArgumentNullException(nameof(inverter));
            this.faultManager = faultManager ?? throw new ArgumentNullException(nameof(faultManager));
            speedPid = new PIDController(SimulationConstants.DefaultKp, SimulationConstants.DefaultKi, SimulationConstants.DefaultKd, 0.01);
            currentPids = new PIDController[2];
            for (int i = 0; i < 2; i++)
                currentPids[i] = new PIDController(SimulationConstants.DefaultCurrentKp, SimulationConstants.DefaultCurrentKi, 0, 0.001);
            iqReference = 0;
            isRunning = false;
            motorType = MotorType.Trapezoidal;
            pwmMode = PwmMode.SixStep;
            useMtpa = true;
            useFieldWeakening = true;
        }

        public void SetSpeedPidGains(double kp, double ki, double kd)
        {
            speedPid.SetGains(kp, ki, kd);
        }

        public void SetSpeedReference(double speedRpm)
        {
            speedPid.SetSetpoint(speedRpm);
        }

        public double GetIqReference()
        {
            return iqReference;
        }

        public void SetMotorType(MotorType type)
        {
            motorType = type;
            motor.SetMotorType(type);
        }

        public void SetPwmMode(PwmMode mode)
        {
            pwmMode = mode;
        }

        public void SetMtpaEnabled(bool enabled)
        {
            useMtpa = enabled;
        }

        public void SetFieldWeakeningEnabled(bool enabled)
        {
            useFieldWeakening = enabled;
        }

        public double[] GetPhaseVoltages(double deltaTime)
        {
            if (!isRunning || faultManager.IsProtectionActive())
                return new double[3];

            double speedError = speedPid.Update(motor.SpeedRpm, deltaTime);
            double torqueDemand = speedError * faultManager.GetDeratingFactor();
            double[] voltages = new double[3];

            if (motorType == MotorType.Trapezoidal)
            {
                int sector = faultManager.GetEffectiveSector(motor.GetCommutationSector());
                double[] currents = motor.GetPhaseCurrents();
                double[] currentRefs = new double[2];
                int[] activePhases = new int[2];
                switch (sector)
                {
                    case 0:
                        currentRefs[0] = torqueDemand; currentRefs[1] = -torqueDemand;
                        activePhases[0] = 0; activePhases[1] = 1;
                        break;
                    case 1:
                        currentRefs[0] = torqueDemand; currentRefs[1] = -torqueDemand;
                        activePhases[0] = 0; activePhases[1] = 2;
                        break;
                    case 2:
                        currentRefs[0] = torqueDemand; currentRefs[1] = -torqueDemand;
                        activePhases[0] = 1; activePhases[1] = 2;
                        break;
                    case 3:
                        currentRefs[0] = torqueDemand; currentRefs[1] = -torqueDemand;
                        activePhases[0] = 1; activePhases[1] = 0;
                        break;
                    case 4:
                        currentRefs[0] = torqueDemand; currentRefs[1] = -torqueDemand;
                        activePhases[0] = 2; activePhases[1] = 0;
                        break;
                    case 5:
                        currentRefs[0] = torqueDemand; currentRefs[1] = -torqueDemand;
                        activePhases[0] = 2; activePhases[1] = 1;
                        break;
                }
                voltages[activePhases[0]] = currentPids[0].Update(currentRefs[0] - currents[activePhases[0]], deltaTime / 10);
                voltages[activePhases[1]] = currentPids[1].Update(currentRefs[1] - currents[activePhases[1]], deltaTime / 10);
                return inverter.ApplyVoltages(voltages, currents, deltaTime, pwmMode == PwmMode.SixStep, sector);
            }
            else // Sinusoidal (FOC)
            {
                double[] dqCurrents = motor.GetDqCurrents();
                double omega = motor.SpeedRpm * 2 * Math.PI / 60.0; // rad/s
                double idRef = 0;
                iqReference = torqueDemand / SimulationConstants.TorqueConstant;

                // MTPA: For surface-mount PMSM, id = 0 maximizes torque per ampere
                if (useMtpa)
                {
                    iqReference = Math.Min(torqueDemand / SimulationConstants.TorqueConstant, SimulationConstants.MaxCurrent);
                    idRef = 0; // No saliency, so id = 0 for MTPA
                }

                // Field Weakening: Adjust id to reduce back-EMF if speed exceeds base speed
                if (useFieldWeakening && motor.SpeedRpm > motor.GetBaseSpeedRpm())
                {
                    double vMax = SimulationConstants.MaxVoltageUtilization * inverter.GetDCLinkVoltage();
                    double L = motor.GetInductance();
                    double backEmf = SimulationConstants.BackEmfConstant * omega;
                    double idMax = -Math.Sqrt(Math.Pow(SimulationConstants.MaxCurrent, 2) - Math.Pow(iqReference, 2));
                    double idFw = -(vMax - backEmf) / (omega * L); // Negative id to reduce flux
                    idRef = Math.Max(idMax, Math.Min(0, idFw)); // Ensure id is negative and within current limit
                    iqReference = Math.Sqrt(Math.Pow(SimulationConstants.MaxCurrent, 2) - Math.Pow(idRef, 2)) * Math.Sign(iqReference);
                }

                // Current control
                double idError = currentPids[0].Update(idRef - dqCurrents[0], deltaTime / 10);
                double iqError = currentPids[1].Update(iqReference - dqCurrents[1], deltaTime / 10);

                // Inverse Park transform
                double theta = motor.RotorAngle * Math.PI / 180.0;
                double v_alpha = idError * Math.Cos(theta) - iqError * Math.Sin(theta);
                double v_beta = idError * Math.Sin(theta) + iqError * Math.Cos(theta);
                voltages[0] = v_alpha;
                voltages[1] = -0.5 * v_alpha + (Math.Sqrt(3) / 2) * v_beta;
                voltages[2] = -0.5 * v_alpha - (Math.Sqrt(3) / 2) * v_beta;

                return inverter.ApplyVoltages(voltages, motor.GetPhaseCurrents(), deltaTime, pwmMode == PwmMode.SixStep, 0);
            }
        }

        public void StopMotor()
        {
            isRunning = false;
            speedPid.Reset();
            foreach (var pid in currentPids)
                pid.Reset();
            inverter.Reset();
        }

        public void StartMotor()
        {
            isRunning = true;
        }
    }
}