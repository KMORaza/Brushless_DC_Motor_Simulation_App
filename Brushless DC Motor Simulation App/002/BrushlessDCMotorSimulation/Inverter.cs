using System;

namespace BrushlessDCMotorSimulation
{
    public enum SwitchingMode { Ideal, Realistic }

    public class Inverter
    {
        private readonly DCLink dcLink;
        private readonly ThermalModel thermalModel;
        private SwitchingMode switchingMode;
        private bool[] switchStates; // 6 switches (3 high-side, 3 low-side)
        private double[] dutyCycles; // Duty cycles for PWM
        private double totalSwitchingLoss;

        public Inverter(DCLink dcLink)
        {
            this.dcLink = dcLink ?? throw new ArgumentNullException(nameof(dcLink));
            this.thermalModel = new ThermalModel();
            switchingMode = SwitchingMode.Ideal;
            switchStates = new bool[6]; // [S1_high, S1_low, S2_high, S2_low, S3_high, S3_low]
            dutyCycles = new double[3]; // Duty cycles for each phase
            totalSwitchingLoss = 0;
        }

        public void SetSwitchingMode(SwitchingMode mode)
        {
            switchingMode = mode;
        }

        public double[] ApplyVoltages(double[] desiredVoltages, double[] phaseCurrents, double deltaTime, bool isSixStep, int sector)
        {
            double[] outputVoltages = new double[3];
            double v_dc = dcLink.Voltage;

            // Convert desired voltages to duty cycles
            for (int i = 0; i < 3; i++)
            {
                double maxVoltage = v_dc / Math.Sqrt(3);
                dutyCycles[i] = (desiredVoltages[i] / maxVoltage + 1) / 2;
                dutyCycles[i] = Math.Max(0, Math.Min(1, dutyCycles[i]));
            }

            double conductionLoss = 0;
            if (isSixStep)
            {
                // 6-step commutation: only two phases active
                bool[] newSwitchStates = new bool[6];
                switch (sector)
                {
                    case 0: newSwitchStates = new bool[] { true, false, false, true, false, false }; break; // A+, B-
                    case 1: newSwitchStates = new bool[] { true, false, false, false, false, true }; break; // A+, C-
                    case 2: newSwitchStates = new bool[] { false, false, true, false, false, true }; break; // B+, C-
                    case 3: newSwitchStates = new bool[] { false, true, true, false, false, false }; break; // B+, A-
                    case 4: newSwitchStates = new bool[] { false, true, false, false, true, false }; break; // C+, A-
                    case 5: newSwitchStates = new bool[] { false, false, false, true, true, false }; break; // C+, B-
                }

                // Apply dead time and calculate switching losses
                for (int i = 0; i < 6; i++)
                {
                    if (newSwitchStates[i] != switchStates[i])
                    {
                        if (switchingMode == SwitchingMode.Realistic)
                        {
                            double current = phaseCurrents[i / 2];
                            double loss = switchStates[i] ? SimulationConstants.MosfetSwitchingEnergyOff : SimulationConstants.MosfetSwitchingEnergyOn;
                            totalSwitchingLoss += loss * Math.Abs(current);
                        }
                    }
                }
                switchStates = newSwitchStates;

                // Calculate output voltages and conduction losses
                for (int i = 0; i < 3; i++)
                {
                    bool highOn = switchStates[i * 2];
                    bool lowOn = switchStates[i * 2 + 1];
                    if (highOn && !lowOn)
                    {
                        outputVoltages[i] = v_dc - SimulationConstants.MosfetOnResistance * phaseCurrents[i];
                        conductionLoss += Math.Pow(phaseCurrents[i], 2) * SimulationConstants.MosfetOnResistance;
                    }
                    else if (!highOn && lowOn)
                    {
                        outputVoltages[i] = -SimulationConstants.MosfetOnResistance * phaseCurrents[i];
                        conductionLoss += Math.Pow(phaseCurrents[i], 2) * SimulationConstants.MosfetOnResistance;
                    }
                    else
                        outputVoltages[i] = 0; // Both off during dead time
                }
            }
            else
            {
                // SVPWM: PWM with dead time
                for (int i = 0; i < 3; i++)
                {
                    double duty = dutyCycles[i];
                    double effectiveDuty = duty;

                    if (switchingMode == SwitchingMode.Realistic)
                    {
                        // Account for dead time distortion
                        double deadTimeFraction = SimulationConstants.DeadTime / (1.0 / SimulationConstants.SwitchingFrequency);
                        effectiveDuty = duty - deadTimeFraction * Math.Sign(phaseCurrents[i]);
                        effectiveDuty = Math.Max(0, Math.Min(1, effectiveDuty));

                        // Switching losses
                        double current = Math.Abs(phaseCurrents[i]);
                        double switchesPerCycle = SimulationConstants.SwitchingFrequency * deltaTime;
                        totalSwitchingLoss += switchesPerCycle * (SimulationConstants.MosfetSwitchingEnergyOn + SimulationConstants.MosfetSwitchingEnergyOff) * current;

                        // Rise/fall time effects
                        double slewTime = SimulationConstants.MosfetRiseTime + SimulationConstants.MosfetFallTime;
                        effectiveDuty *= (1 - slewTime * SimulationConstants.SwitchingFrequency);
                    }

                    // Calculate output voltage and conduction loss
                    outputVoltages[i] = (effectiveDuty * 2 - 1) * (v_dc / 2);
                    outputVoltages[i] -= SimulationConstants.MosfetOnResistance * phaseCurrents[i];
                    conductionLoss += Math.Pow(phaseCurrents[i], 2) * SimulationConstants.MosfetOnResistance * effectiveDuty;
                }
            }

            // Update thermal model
            thermalModel.Update(totalSwitchingLoss / deltaTime + conductionLoss, deltaTime);

            return outputVoltages;
        }

        public void Reset()
        {
            switchStates = new bool[6];
            dutyCycles = new double[3];
            totalSwitchingLoss = 0;
            thermalModel.Reset();
        }

        public double GetTotalSwitchingLoss()
        {
            return totalSwitchingLoss;
        }

        public double GetTemperature()
        {
            return thermalModel.Temperature;
        }
    }
}