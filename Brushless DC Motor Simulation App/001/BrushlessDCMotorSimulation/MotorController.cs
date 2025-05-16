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

        public double[] GetPhaseVoltages(double deltaTime)
        {
            if (!isRunning || faultManager.IsProtectionActive())
                return new double[3];

            double speedError = speedPid.Update(motor.SpeedRpm, deltaTime);
            iqReference = Math.Max(-10, Math.Min(10, speedError * faultManager.GetDeratingFactor()));
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
                        currentRefs[0] = iqReference; currentRefs[1] = -iqReference;
                        activePhases[0] = 0; activePhases[1] = 1;
                        break;
                    case 1:
                        currentRefs[0] = iqReference; currentRefs[1] = -iqReference;
                        activePhases[0] = 0; activePhases[1] = 2;
                        break;
                    case 2:
                        currentRefs[0] = iqReference; currentRefs[1] = -iqReference;
                        activePhases[0] = 1; activePhases[1] = 2;
                        break;
                    case 3:
                        currentRefs[0] = iqReference; currentRefs[1] = -iqReference;
                        activePhases[0] = 1; activePhases[1] = 0;
                        break;
                    case 4:
                        currentRefs[0] = iqReference; currentRefs[1] = -iqReference;
                        activePhases[0] = 2; activePhases[1] = 0;
                        break;
                    case 5:
                        currentRefs[0] = iqReference; currentRefs[1] = -iqReference;
                        activePhases[0] = 2; activePhases[1] = 1;
                        break;
                }
                voltages[activePhases[0]] = currentPids[0].Update(currentRefs[0] - currents[activePhases[0]], deltaTime / 10);
                voltages[activePhases[1]] = currentPids[1].Update(currentRefs[1] - currents[activePhases[1]], deltaTime / 10);
                return inverter.ApplyVoltages(voltages, currents, deltaTime, pwmMode == PwmMode.SixStep, sector);
            }
            else
            {
                double[] dqCurrents = motor.GetDqCurrents();
                double idRef = 0;
                double idError = currentPids[0].Update(idRef - dqCurrents[0], deltaTime / 10);
                double iqError = currentPids[1].Update(iqReference - dqCurrents[1], deltaTime / 10);
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