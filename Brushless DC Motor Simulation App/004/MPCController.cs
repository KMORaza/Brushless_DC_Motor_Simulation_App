using System;

namespace BrushlessDCMotorSimulation
{
    public class MPCController
    {
        private readonly BLDCMotor motor;
        private readonly Inverter inverter;
        private readonly double dt; // Control time step (s)
        private int N; // Prediction horizon
        private double Q_speed; // Weight for speed error
        private double Q_iq; // Weight for q-axis current error
        private double Q_id; // Weight for d-axis current error
        private double R; // Weight for control effort
        private readonly double L; // Inductance (H)
        private readonly double R_s; // Phase resistance (Ohm)
        private readonly double psi; // Flux linkage (V.s/rad)
        private readonly double J; // Moment of inertia (kg.m²)
        private readonly double B; // Friction coefficient (Nm/(rad/s))
        private readonly double kt; // Torque constant (Nm/A)
        private readonly double maxVoltage; // Voltage limit (V)
        private readonly double maxCurrent; // Current limit (A)

        public MPCController(BLDCMotor motor, Inverter inverter, double dt)
        {
            this.motor = motor ?? throw new ArgumentNullException(nameof(motor));
            this.inverter = inverter ?? throw new ArgumentNullException(nameof(inverter));
            this.dt = dt;
            N = 5; // Default prediction horizon
            Q_speed = 100.0; // Default weight for speed tracking
            Q_iq = 10.0; // Default weight for q-axis current
            Q_id = 10.0; // Default weight for d-axis current
            R = 0.01; // Default weight for control effort
            L = SimulationConstants.MutualInductance;
            R_s = SimulationConstants.PhaseResistance;
            psi = SimulationConstants.BackEmfConstant;
            J = SimulationConstants.MomentOfInertia;
            B = SimulationConstants.FrictionCoefficient; // Used in state-space matrix A
            kt = SimulationConstants.TorqueConstant;
            maxVoltage = SimulationConstants.MaxVoltageUtilization * SimulationConstants.SupplyVoltage;
            maxCurrent = SimulationConstants.MaxCurrent;
        }

        // Setters for tuning MPC parameters
        public void SetPredictionHorizon(int horizon)
        {
            if (horizon < 1)
                throw new ArgumentException("Prediction horizon must be at least 1.");
            N = horizon;
        }

        public void SetSpeedWeight(double weight)
        {
            if (weight <= 0)
                throw new ArgumentException("Speed weight must be positive.");
            Q_speed = weight;
        }

        public void SetIqWeight(double weight)
        {
            if (weight <= 0)
                throw new ArgumentException("Iq weight must be positive.");
            Q_iq = weight;
        }

        public void SetIdWeight(double weight)
        {
            if (weight <= 0)
                throw new ArgumentException("Id weight must be positive.");
            Q_id = weight;
        }

        public void SetControlWeight(double weight)
        {
            if (weight <= 0)
                throw new ArgumentException("Control weight must be positive.");
            R = weight;
        }

        public double[] ComputeControl(double speedRef, double loadTorque, double[] dqCurrents, double omega, double deltaTime)
        {
            // Current state: [i_d, i_q, omega]
            double[] state = new double[] { dqCurrents[0], dqCurrents[1], omega };

            // References: i_d = 0 (MTPA for surface-mount PMSM), i_q from torque, speed
            double torqueRef = (speedRef - omega * 60.0 / (2 * Math.PI)) * 0.5; // Speed error to torque (RPM to rad/s)
            double iqRef = torqueRef / kt;
            iqRef = Math.Max(-maxCurrent, Math.Min(maxCurrent, iqRef));
            double idRef = 0.0; // MTPA

            // Discretized state-space model (simplified dq model)
            // x[k+1] = A*x[k] + B_matrix*u[k] + d[k]
            // x = [i_d, i_q, omega], u = [v_d, v_q]
            double[,] A = new double[,]
            {
                { 1 - dt * R_s / L, 0, 0 },
                { 0, 1 - dt * R_s / L, 0 },
                { 0, 3 * kt * dt / (2 * J), 1 - B * dt / J } // B is friction coefficient
            };
            double[,] B_matrix = new double[,]
            {
                { dt / L, 0 },
                { 0, dt / L },
                { 0, 0 }
            };
            double[] d = new double[]
            {
                -dt * psi * omega / L, // Back-EMF term for i_d
                0,
                -loadTorque * dt / J // Load torque disturbance
            };

            // Initialize optimization variables
            double[] u_opt = new double[2]; // Optimal [v_d, v_q]
            double minCost = double.MaxValue;
            int numSamples = 10; // Grid search samples per control input

            // Simple grid search over control inputs (v_d, v_q)
            for (double vd = -maxVoltage; vd <= maxVoltage; vd += 2 * maxVoltage / numSamples)
            {
                for (double vq = -maxVoltage; vq <= maxVoltage; vq += 2 * maxVoltage / numSamples)
                {
                    // Check voltage constraint: sqrt(v_d^2 + v_q^2) <= maxVoltage
                    if (Math.Sqrt(vd * vd + vq * vq) > maxVoltage)
                        continue;

                    double cost = 0;
                    double[] x = (double[])state.Clone();

                    // Predict states over horizon
                    for (int k = 0; k < N; k++)
                    {
                        // Compute next state
                        double[] u = k == 0 ? new double[] { vd, vq } : new double[] { 0, 0 }; // Apply control only at k=0
                        double[] x_next = new double[3];
                        for (int i = 0; i < 3; i++)
                        {
                            x_next[i] = d[i];
                            for (int j = 0; j < 3; j++)
                                x_next[i] += A[i, j] * x[j];
                            if (i < 2)
                                for (int j = 0; j < 2; j++)
                                    x_next[i] += B_matrix[i, j] * u[j];
                        }

                        // Check current constraint
                        if (Math.Sqrt(x_next[0] * x_next[0] + x_next[1] * x_next[1]) > maxCurrent)
                        {
                            cost = double.MaxValue;
                            break;
                        }

                        // Compute cost
                        double speedError = x_next[2] - (speedRef * 2 * Math.PI / 60.0); // Speed in rad/s
                        double idError = x_next[0] - idRef;
                        double iqError = x_next[1] - iqRef;
                        cost += Q_speed * speedError * speedError +
                                Q_id * idError * idError +
                                Q_iq * iqError * iqError;
                        if (k == 0)
                            cost += R * (vd * vd + vq * vq);

                        x = x_next;
                    }

                    if (cost < minCost)
                    {
                        minCost = cost;
                        u_opt[0] = vd;
                        u_opt[1] = vq;
                    }
                }
            }

            return u_opt; // Return optimal [v_d, v_q]
        }
    }
}