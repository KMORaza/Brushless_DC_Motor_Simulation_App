using System;

namespace BrushlessDCMotorSimulation
{
    public class PIDController
    {
        private double kp, ki, kd;
        private double integral, previousError;
        private double setpoint;
        private readonly double dt;
        private readonly double integralLimit;

        public PIDController(double kp, double ki, double kd, double dt)
        {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.dt = dt;
            integral = 0;
            previousError = 0;
            setpoint = 0;
            integralLimit = 10;
        }

        public void SetGains(double kp, double ki, double kd)
        {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public void SetSetpoint(double setpoint)
        {
            this.setpoint = setpoint;
        }

        public double Update(double actual, double dt)
        {
            double error = setpoint - actual;
            integral += error * dt;
            integral = Math.Max(-integralLimit, Math.Min(integralLimit, integral));
            double derivative = (error - previousError) / dt;
            previousError = error;
            return kp * error + ki * integral + kd * derivative;
        }

        public void Reset()
        {
            integral = 0;
            previousError = 0;
        }
    }
}