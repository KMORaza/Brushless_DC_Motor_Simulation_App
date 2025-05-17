using System;

namespace BrushlessDCMotorSimulation
{
    public enum PwmMode { SixStep, SVPWM }
    public enum ControlMode { PID, MPC }

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
        private readonly MPCController mpcController;
        private readonly ParameterEstimator parameterEstimator;
        public MPCController MpcController => mpcController; // Public access for tuning
        private double iqReference;
        private double speedReference; // Stores speed setpoint to avoid GetSetpoint
        private bool isRunning;
        private MotorType motorType;
        private PwmMode pwmMode;
        private ControlMode controlMode;
        private bool useMtpa;
        private bool useFieldWeakening;
        private bool useAdaptiveControl;

        public MotorController(BLDCMotor motor, Inverter inverter, FaultManager faultManager)
        {
            this.motor = motor ?? throw new ArgumentNullException(nameof(motor));
            this.inverter = inverter ?? throw new ArgumentNullException(nameof(inverter));
            this.faultManager = faultManager ?? throw new ArgumentNullException(nameof(faultManager));
            speedPid = new PIDController(SimulationConstants.DefaultKp, SimulationConstants.DefaultKi, SimulationConstants.DefaultKd, 0.01);
            currentPids = new PIDController[2];
            for (int i = 0; i < 2; i++)
                currentPids[i] = new PIDController(SimulationConstants.DefaultCurrentKp, SimulationConstants.DefaultCurrentKi, 0, 0.001);
            mpcController = new MPCController(motor, inverter, 0.001);
            parameterEstimator = new ParameterEstimator(motor);
            iqReference = 0;
            speedReference = 0;
            isRunning = false;
            motorType = MotorType.Trapezoidal;
            pwmMode = PwmMode.SixStep;
            controlMode = ControlMode.PID;
            useMtpa = true;
            useFieldWeakening = true;
            useAdaptiveControl = false;
        }

        public void SetSpeedPidGains(double kp, double ki, double kd)
        {
            speedPid.SetGains(kp, ki, kd);
        }

        public void SetSpeedReference(double speedRpm)
        {
            speedPid.SetSetpoint(speedRpm);
            speedReference = speedRpm; // Store the setpoint
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

        public void SetControlMode(ControlMode mode)
        {
            controlMode = mode;
        }

        public void SetMtpaEnabled(bool enabled)
        {
            useMtpa = enabled;
        }

        public void SetFieldWeakeningEnabled(bool enabled)
        {
            useFieldWeakening = enabled;
        }

        public void SetAdaptiveControlEnabled(bool enabled)
        {
            useAdaptiveControl = enabled;
        }

        public ParameterEstimator ParameterEstimator => parameterEstimator;

        private void UpdateAdaptiveGains()
        {
            if (!useAdaptiveControl || controlMode != ControlMode.PID) return;

            double Rs = parameterEstimator.GetEstimatedResistance();
            double L = parameterEstimator.GetEstimatedInductance();

            // Adaptive gain adjustment
            double Rs_nominal = SimulationConstants.PhaseResistance;
            double L_nominal = SimulationConstants.MutualInductance;
            double Rs_factor = Rs_nominal / Rs; // Decrease gains if resistance increases
            double L_factor = L_nominal / L; // Adjust based on inductance

            // Speed PID
            var (kp_s, ki_s, kd_s) = speedPid.GetGains();
            speedPid.UpdateGains(
                SimulationConstants.DefaultKp * Rs_factor * 0.8,
                SimulationConstants.DefaultKi * Rs_factor * L_factor * 0.9,
                SimulationConstants.DefaultKd * Rs_factor
            );

            // Current PIDs
            for (int i = 0; i < 2; i++)
            {
                var (kp_c, ki_c, kd_c) = currentPids[i].GetGains();
                currentPids[i].UpdateGains(
                    SimulationConstants.DefaultCurrentKp * L_factor * 0.9,
                    SimulationConstants.DefaultCurrentKi * Rs_factor * L_factor,
                    0
                );
            }
        }

        public double[] GetPhaseVoltages(double deltaTime)
        {
            if (!isRunning || faultManager.IsProtectionActive())
                return new double[3];

            // Update parameter estimation
            double[] voltages = new double[3];
            double[] currents = motor.GetPhaseCurrents();
            parameterEstimator.Update(voltages, currents, deltaTime);
            UpdateAdaptiveGains();

            double speedError = speedPid.Update(motor.SpeedRpm, deltaTime);
            double torqueDemand = speedError * faultManager.GetDeratingFactor();

            if (motorType == MotorType.Trapezoidal)
            {
                int sector = faultManager.GetEffectiveSector(motor.GetCommutationSector());
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

                if (controlMode == ControlMode.MPC)
                {
                    // Use MPC to compute v_d, v_q
                    double[] dqVoltages = mpcController.ComputeControl(speedReference, SimulationConstants.DefaultLoadTorque, dqCurrents, omega, deltaTime / 10);
                    double v_d = dqVoltages[0];
                    double v_q = dqVoltages[1];

                    // Inverse Park transform
                    double theta = motor.RotorAngle * Math.PI / 180.0;
                    double v_alpha = v_d * Math.Cos(theta) - v_q * Math.Sin(theta);
                    double v_beta = v_d * Math.Sin(theta) + v_q * Math.Cos(theta);
                    voltages[0] = v_alpha;
                    voltages[1] = -0.5 * v_alpha + (Math.Sqrt(3) / 2) * v_beta;
                    voltages[2] = -0.5 * v_alpha - (Math.Sqrt(3) / 2) * v_beta;

                    iqReference = dqCurrents[1]; // For UI display
                }
                else // PID control
                {
                    double idRef = 0;
                    iqReference = torqueDemand / SimulationConstants.TorqueConstant;

                    // MTPA: For surface-mount PMSM, id = 0 maximizes torque per ampere
                    if (useMtpa)
                    {
                        iqReference = Math.Min(torqueDemand / SimulationConstants.TorqueConstant, SimulationConstants.MaxCurrent);
                        idRef = 0;
                    }

                    // Field Weakening
                    if (useFieldWeakening && motor.SpeedRpm > motor.GetBaseSpeedRpm())
                    {
                        double vMax = SimulationConstants.MaxVoltageUtilization * inverter.GetDCLinkVoltage();
                        double L = motor.GetInductance();
                        double backEmf = SimulationConstants.BackEmfConstant * omega;
                        double idMax = -Math.Sqrt(Math.Pow(SimulationConstants.MaxCurrent, 2) - Math.Pow(iqReference, 2));
                        double idFw = -(vMax - backEmf) / (omega * L);
                        idRef = Math.Max(idMax, Math.Min(0, idFw));
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
                }

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