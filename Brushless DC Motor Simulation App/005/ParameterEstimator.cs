using System;

namespace BrushlessDCMotorSimulation
{
    public class ParameterEstimator
    {
        private double[] theta; // Estimated parameters [R_s, L]
        private double[,] P; // Covariance matrix
        private readonly double lambda; // Forgetting factor
        private readonly BLDCMotor motor;

        public ParameterEstimator(BLDCMotor motor)
        {
            this.motor = motor ?? throw new ArgumentNullException(nameof(motor));
            theta = new double[] { SimulationConstants.PhaseResistance, SimulationConstants.MutualInductance };
            P = new double[,] { { 1e3, 0 }, { 0, 1e3 } }; // Initial covariance
            lambda = 0.99; // Forgetting factor for RLS
        }

        public void Update(double[] voltages, double[] currents, double deltaTime)
        {
            double[] backEmf = motor.GetBackEMF();
            for (int i = 0; i < 3; i++)
            {
                if (Math.Abs(currents[i]) < 0.1) continue; // Avoid estimation with low current

                // Compute dI/dt
                double di_dt = (currents[i] - GetPreviousCurrent(i)) / deltaTime;

                // Regression vector: [I, dI/dt]
                double[] phi = new double[] { currents[i], di_dt };

                // Measurement: V - E
                double y = voltages[i] - backEmf[i];

                // RLS update
                double[] K = new double[2]; // Gain vector
                double phiT_P_phi = 0;
                for (int j = 0; j < 2; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < 2; k++)
                        sum += phi[j] * P[j, k];
                    K[j] = sum;
                    phiT_P_phi += sum * phi[j];
                }
                double denom = lambda + phiT_P_phi;
                for (int j = 0; j < 2; j++)
                    K[j] /= denom;

                // Update theta
                double error = y - (phi[0] * theta[0] + phi[1] * theta[1]);
                for (int j = 0; j < 2; j++)
                    theta[j] += K[j] * error;

                // Update covariance
                double[,] P_new = new double[2, 2];
                for (int j = 0; j < 2; j++)
                    for (int k = 0; k < 2; k++)
                        P_new[j, k] = (P[j, k] - K[j] * phi[k] * P[j, k]) / lambda;
                P = P_new;
            }

            // Ensure parameters stay within reasonable bounds
            theta[0] = Math.Max(0.1, Math.Min(10.0, theta[0])); // R_s
            theta[1] = Math.Max(0.0001, Math.Min(0.01, theta[1])); // L
        }

        public double GetEstimatedResistance() => theta[0];
        public double GetEstimatedInductance() => theta[1];

        private double[] previousCurrents = new double[3];
        private double GetPreviousCurrent(int phase)
        {
            double current = previousCurrents[phase];
            previousCurrents[phase] = motor.GetPhaseCurrents()[phase];
            return current;
        }
    }
}