using System;

namespace BrushlessDCMotorSimulation
{
    public class SimulationEngine
    {
        private readonly BLDCMotor motor;
        private readonly MotorController controller;
        private readonly Inverter inverter;
        private readonly DCLink dcLink;
        private readonly FaultManager faultManager;
        private double loadTorque;
        private double speedReference;

        public SimulationEngine(BLDCMotor motor, MotorController controller, Inverter inverter, DCLink dcLink, FaultManager faultManager)
        {
            this.motor = motor ?? throw new ArgumentNullException(nameof(motor));
            this.controller = controller ?? throw new ArgumentNullException(nameof(controller));
            this.inverter = inverter ?? throw new ArgumentNullException(nameof(inverter));
            this.dcLink = dcLink ?? throw new ArgumentNullException(nameof(dcLink));
            this.faultManager = faultManager ?? throw new ArgumentNullException(nameof(faultManager));
            loadTorque = SimulationConstants.DefaultLoadTorque;
            speedReference = 1000;
        }

        public void SetLoadTorque(double torque)
        {
            if (torque < 0 || torque > 0.1)
                throw new ArgumentException("Load torque must be between 0 and 0.1 Nm.");
            loadTorque = torque;
        }

        public void SetSpeedReference(double speedRpm)
        {
            speedReference = Math.Max(0, Math.Min(3000, speedRpm));
            controller.SetSpeedReference(speedReference);
        }

        public void SetSpeedPidGains(double kp, double ki, double kd)
        {
            controller.SetSpeedPidGains(kp, ki, kd);
        }

        public void Update(double deltaTime)
        {
            try
            {
                // Use finer sub-steps for inverter, DC link, and fault dynamics
                int subSteps = 100; // 100 µs sub-steps for 10 ms
                double subDeltaTime = deltaTime / subSteps;

                for (int i = 0; i < subSteps; i++)
                {
                    double[] voltages = controller.GetPhaseVoltages(subDeltaTime);
                    double[] currents = motor.GetPhaseCurrents();
                    faultManager.Update(currents, dcLink.Voltage, subDeltaTime);
                    dcLink.Update(currents, subDeltaTime);
                    motor.ApplyVoltages(voltages, subDeltaTime);
                    motor.Update(subDeltaTime, loadTorque);
                }
            }
            catch (Exception ex)
            {
                controller.StopMotor();
                throw new InvalidOperationException("Simulation error: " + ex.Message, ex);
            }
        }
    }
}