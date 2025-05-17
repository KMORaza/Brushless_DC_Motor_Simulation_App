using System;

namespace BrushlessDCMotorSimulation
{
    public enum MotorType { Trapezoidal, Sinusoidal }

    public class BLDCMotor
    {
        public double RotorAngle { get; private set; } // Degrees
        public double SpeedRpm { get; private set; } // RPM
        private double angularVelocityRadPerSec; // rad/s
        private double torque; // Nm
        private readonly double momentOfInertia;
        private readonly double frictionCoefficient;
        private readonly double torqueConstant;
        private readonly double backEmfConstant;
        private readonly double phaseResistance;
        private readonly double mutualInductance;
        private double[] phaseCurrents;
        private MotorType motorType;
        private int phaseLossIndex = -1;
        private (int, int)? shortCircuitPhases = null;
        private double shortCircuitResistance = SimulationConstants.ShortCircuitResistance;

        public BLDCMotor()
        {
            RotorAngle = 0;
            SpeedRpm = 0;
            angularVelocityRadPerSec = 0;
            torque = 0;
            momentOfInertia = SimulationConstants.MomentOfInertia;
            frictionCoefficient = SimulationConstants.FrictionCoefficient;
            torqueConstant = SimulationConstants.TorqueConstant;
            backEmfConstant = SimulationConstants.BackEmfConstant;
            phaseResistance = SimulationConstants.PhaseResistance;
            mutualInductance = SimulationConstants.MutualInductance;
            phaseCurrents = new double[3];
            motorType = MotorType.Trapezoidal;
        }

        public double GetBaseSpeedRpm()
        {
            return SimulationConstants.BaseSpeedRpm;
        }

        public double GetInductance()
        {
            double maxCurrent = Math.Max(Math.Abs(phaseCurrents[0]),
                Math.Max(Math.Abs(phaseCurrents[1]), Math.Abs(phaseCurrents[2])));
            double saturationFactor = 1.0 / (1.0 + Math.Pow(maxCurrent / SimulationConstants.SaturationCurrent, 2));
            return SimulationConstants.MinInductance +
                (mutualInductance - SimulationConstants.MinInductance) * saturationFactor;
        }

        public void SetMotorType(MotorType type)
        {
            motorType = type;
        }

        public void SetPhaseLoss(int phase)
        {
            if (phase >= 0 && phase < 3)
                phaseLossIndex = phase;
            else
                phaseLossIndex = -1;
        }

        public void SetShortCircuit(int phase1, int phase2)
        {
            if (phase1 >= 0 && phase1 < 3 && phase2 >= 0 && phase2 < 3 && phase1 != phase2)
                shortCircuitPhases = (phase1, phase2);
            else
                shortCircuitPhases = null;
        }

        public void ClearFaults()
        {
            phaseLossIndex = -1;
            shortCircuitPhases = null;
        }

        public void ApplyVoltages(double[] voltages, double deltaTime)
        {
            if (voltages.Length != 3)
                throw new ArgumentException("Voltages array must have 3 elements.");

            double[] backEmf = GetBackEMF();
            double[] effectiveResistances = new double[3] { phaseResistance, phaseResistance, phaseResistance };
            double effectiveInductance = GetInductance();

            if (shortCircuitPhases.HasValue)
            {
                var (p1, p2) = shortCircuitPhases.Value;
                effectiveResistances[p1] = shortCircuitResistance;
                effectiveResistances[p2] = shortCircuitResistance;
                double v_diff = voltages[p1] - voltages[p2];
                phaseCurrents[p1] = v_diff / (2 * shortCircuitResistance);
                phaseCurrents[p2] = -phaseCurrents[p1];
            }

            for (int i = 0; i < 3; i++)
            {
                if (i == phaseLossIndex || shortCircuitPhases.HasValue &&
                    (i == shortCircuitPhases.Value.Item1 || i == shortCircuitPhases.Value.Item2))
                    continue;

                double di_dt = (voltages[i] - effectiveResistances[i] * phaseCurrents[i] - backEmf[i]) / effectiveInductance;
                phaseCurrents[i] += di_dt * deltaTime;
                if (Math.Abs(phaseCurrents[i]) > SimulationConstants.MaxCurrent)
                    phaseCurrents[i] = Math.Sign(phaseCurrents[i]) * SimulationConstants.MaxCurrent;
            }

            if (phaseLossIndex >= 0)
                phaseCurrents[phaseLossIndex] = 0;
        }

        public double[] GetBackEMF()
        {
            double[] backEmf = new double[3];
            double omega = angularVelocityRadPerSec;
            for (int i = 0; i < 3; i++)
            {
                backEmf[i] = backEmfConstant * omega * GetBackEMFPhase(RotorAngle - i * 120.0, i);
            }
            return backEmf;
        }

        public double GetBackEMFPhase(double angle, int phase)
        {
            angle = (angle % 360.0 + 360.0) % 360.0;
            if (motorType == MotorType.Trapezoidal)
            {
                double sector = (angle % 60.0) / 60.0;
                if (angle < 60 || angle >= 300) return sector;
                if (angle >= 60 && angle < 120) return 1.0;
                if (angle >= 120 && angle < 180) return 1.0 - sector;
                if (angle >= 180 && angle < 240) return -sector;
                if (angle >= 240 && angle < 300) return -1.0;
                return -(1.0 - sector);
            }
            return Math.Sin(angle * Math.PI / 180.0 + phase * 2 * Math.PI / 3);
        }

        public double[] GetPhaseCurrents()
        {
            return (double[])phaseCurrents.Clone();
        }

        public double[] GetDqCurrents()
        {
            double i_alpha = (2.0 / 3.0) * (phaseCurrents[0] - 0.5 * phaseCurrents[1] - 0.5 * phaseCurrents[2]);
            double i_beta = (2.0 / 3.0) * (Math.Sqrt(3) / 2 * (phaseCurrents[1] - phaseCurrents[2]));
            double theta = RotorAngle * Math.PI / 180.0;
            double i_d = i_alpha * Math.Cos(theta) + i_beta * Math.Sin(theta);
            double i_q = -i_alpha * Math.Sin(theta) + i_beta * Math.Cos(theta);
            return new double[] { i_d, i_q };
        }

        public double GetElectromagneticTorque()
        {
            double baseTorque = torqueConstant * (phaseCurrents[0] + phaseCurrents[1] + phaseCurrents[2]);
            double rippleTorque = GetRippleTorque();
            return baseTorque + rippleTorque;
        }

        public double GetFrictionTorque()
        {
            return frictionCoefficient * angularVelocityRadPerSec;
        }

        public double GetCoggingTorque()
        {
            double angleRad = RotorAngle * Math.PI / 180.0;
            double frequency = SimulationConstants.SlotsPerPole;
            return SimulationConstants.CoggingTorqueAmplitude * Math.Sin(frequency * angleRad);
        }

        public double GetRippleTorque()
        {
            double angleRad = RotorAngle * Math.PI / 180.0;
            double currentMagnitude = Math.Sqrt(phaseCurrents[0] * phaseCurrents[0] +
                                                phaseCurrents[1] * phaseCurrents[1] +
                                                phaseCurrents[2] * phaseCurrents[2]);
            double harmonic = SimulationConstants.RippleHarmonicOrder;
            return SimulationConstants.RippleTorqueAmplitude * currentMagnitude *
                Math.Sin(harmonic * angleRad) / SimulationConstants.MaxCurrent;
        }

        public void Update(double deltaTime, double loadTorque)
        {
            double Te = GetElectromagneticTorque();
            double Tc = GetCoggingTorque();
            torque = Te + Tc - loadTorque - GetFrictionTorque();
            double omega = angularVelocityRadPerSec;
            double theta = RotorAngle * Math.PI / 180.0;
            Func<double, double, double, double> dOmega_dt =
                (t, w, th) => (Te + Tc - loadTorque - frictionCoefficient * w) / momentOfInertia;
            Func<double, double, double, double> dTheta_dt = (t, w, th) => w;
            double k1_omega = dOmega_dt(0, omega, theta);
            double k1_theta = dTheta_dt(0, omega, theta);
            double k2_omega = dOmega_dt(deltaTime / 2, omega + k1_omega * deltaTime / 2, theta + k1_theta * deltaTime / 2);
            double k2_theta = dTheta_dt(deltaTime / 2, omega + k1_omega * deltaTime / 2, theta + k1_theta * deltaTime / 2);
            double k3_omega = dOmega_dt(deltaTime / 2, omega + k2_omega * deltaTime / 2, theta + k2_theta * deltaTime / 2);
            double k3_theta = dTheta_dt(deltaTime / 2, omega + k2_omega * deltaTime / 2, theta + k2_theta * deltaTime / 2);
            double k4_omega = dOmega_dt(deltaTime, omega + k3_omega * deltaTime, theta + k3_theta * deltaTime);
            double k4_theta = dTheta_dt(deltaTime, omega + k3_omega * deltaTime, theta + k3_theta * deltaTime);
            angularVelocityRadPerSec += (deltaTime / 6.0) * (k1_omega + 2 * k2_omega + 2 * k3_omega + k4_omega);
            theta += (deltaTime / 6.0) * (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta);
            RotorAngle = (theta * 180.0 / Math.PI) % 360.0;
            if (RotorAngle < 0) RotorAngle += 360.0;
            SpeedRpm = (angularVelocityRadPerSec * 60.0) / (2 * Math.PI);
        }

        public int GetCommutationSector()
        {
            return (int)(RotorAngle % 360.0 / 60.0);
        }

        public void Stop()
        {
            angularVelocityRadPerSec = 0;
            SpeedRpm = 0;
            torque = 0;
            phaseCurrents = new double[3];
        }
    }
}