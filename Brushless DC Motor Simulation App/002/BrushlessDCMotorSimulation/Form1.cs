using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace BrushlessDCMotorSimulation
{
    public partial class Form1 : Form
    {
        private readonly SimulationEngine simEngine;
        private readonly BLDCMotor motor;
        private readonly MotorController controller;
        private readonly Inverter inverter;
        private readonly DCLink dcLink;
        private readonly FaultManager faultManager;
        private Timer simulationTimer;
        private Label lblSpeed, lblPhaseA, lblPhaseB, lblPhaseC, lblIqRef, lblIqAct, lblTorque, lblStatus, lblDCLink;
        private Label lblFaultPhaseLoss, lblFaultShortCircuit, lblFaultSensor, lblFaultOvervoltage, lblFaultOvercurrent;
        private CustomPanel rotorPanel, bemfPanel;
        private ComboBox cmbMotorType, cmbPwmMode, cmbSwitchingMode;
        private TrackBar loadTorqueTrackBar, speedRefTrackBar;
        private bool isRunning;
        private bool isPaused;

        // Custom Panel class to enable double buffering
        private class CustomPanel : Panel
        {
            public CustomPanel()
            {
                DoubleBuffered = true;
            }
        }

        public Form1()
        {
            motor = new BLDCMotor();
            dcLink = new DCLink();
            inverter = new Inverter(dcLink);
            faultManager = new FaultManager(motor, inverter, dcLink);
            controller = new MotorController(motor, inverter, faultManager);
            simEngine = new SimulationEngine(motor, controller, inverter, dcLink, faultManager);
            isRunning = false;
            isPaused = false;
            SetupUI();
        }

        private void SetupUI()
        {
            Text = "BLDC Motor Simulator";
            Size = new Size(900, 800);
            BackColor = Color.Black;
            Font = new Font("MS Sans Serif", 8f);
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;

            // Control GroupBox
            GroupBox grpControl = new GroupBox
            {
                Text = "MOTOR CONTROL",
                Location = new Point(10, 10),
                Size = new Size(240, 350),
                BackColor = Color.Black,
                ForeColor = Color.White, // White title
                FlatStyle = FlatStyle.Standard
            };

            Button btnStartStop = new Button
            {
                Text = "Start",
                Location = new Point(20, 20),
                Size = new Size(75, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnStartStop.Click += BtnStartStop_Click;
            grpControl.Controls.Add(btnStartStop);

            Button btnPause = new Button
            {
                Text = "Pause",
                Location = new Point(100, 20),
                Size = new Size(75, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnPause.Click += BtnPause_Click;
            grpControl.Controls.Add(btnPause);

            Button btnRestart = new Button
            {
                Text = "Restart",
                Location = new Point(180, 20),
                Size = new Size(75, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnRestart.Click += BtnRestart_Click;
            grpControl.Controls.Add(btnRestart);

            Label lblSpeedRef = new Label
            {
                Text = "Speed Ref (RPM):",
                Location = new Point(20, 65),
                Size = new Size(80, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            speedRefTrackBar = new TrackBar
            {
                Location = new Point(100, 60),
                Size = new Size(120, 45),
                Minimum = 0,
                Maximum = 3000,
                Value = 1000,
                TickStyle = TickStyle.BottomRight,
                BackColor = SystemColors.Control
            };
            speedRefTrackBar.Scroll += (s, e) => simEngine.SetSpeedReference(speedRefTrackBar.Value);
            grpControl.Controls.Add(lblSpeedRef);
            grpControl.Controls.Add(speedRefTrackBar);

            Label lblMotorType = new Label
            {
                Text = "Motor Type:",
                Location = new Point(20, 110),
                Size = new Size(80, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            cmbMotorType = new ComboBox
            {
                Location = new Point(100, 110),
                Size = new Size(120, 21),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = SystemColors.Window,
                ForeColor = Color.Black,
                FlatStyle = FlatStyle.Standard
            };
            cmbMotorType.Items.AddRange(new object[] { "Trapezoidal (BLDC)", "Sinusoidal (FOC)" });
            cmbMotorType.SelectedIndex = 0;
            cmbMotorType.SelectedIndexChanged += (s, e) =>
                controller.SetMotorType(cmbMotorType.SelectedIndex == 0 ? MotorType.Trapezoidal : MotorType.Sinusoidal);
            grpControl.Controls.Add(lblMotorType);
            grpControl.Controls.Add(cmbMotorType);

            Label lblPwmMode = new Label
            {
                Text = "PWM Mode:",
                Location = new Point(20, 140),
                Size = new Size(80, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            cmbPwmMode = new ComboBox
            {
                Location = new Point(100, 140),
                Size = new Size(120, 21),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = SystemColors.Window,
                ForeColor = Color.Black,
                FlatStyle = FlatStyle.Standard
            };
            cmbPwmMode.Items.AddRange(new object[] { "6-Step PWM", "SVPWM" });
            cmbPwmMode.SelectedIndex = 0;
            cmbPwmMode.SelectedIndexChanged += (s, e) =>
                controller.SetPwmMode(cmbPwmMode.SelectedIndex == 0 ? PwmMode.SixStep : PwmMode.SVPWM);
            grpControl.Controls.Add(lblPwmMode);
            grpControl.Controls.Add(cmbPwmMode);

            Label lblSwitchingMode = new Label
            {
                Text = "Switching Mode:",
                Location = new Point(20, 170),
                Size = new Size(80, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            cmbSwitchingMode = new ComboBox
            {
                Location = new Point(100, 170),
                Size = new Size(120, 21),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = SystemColors.Window,
                ForeColor = Color.Black,
                FlatStyle = FlatStyle.Standard
            };
            cmbSwitchingMode.Items.AddRange(new object[] { "Ideal", "Realistic" });
            cmbSwitchingMode.SelectedIndex = 0;
            cmbSwitchingMode.SelectedIndexChanged += (s, e) =>
                inverter.SetSwitchingMode(cmbSwitchingMode.SelectedIndex == 0 ? SwitchingMode.Ideal : SwitchingMode.Realistic);
            grpControl.Controls.Add(lblSwitchingMode);
            grpControl.Controls.Add(cmbSwitchingMode);

            Label lblLoadTorque = new Label
            {
                Text = "Load Torque:",
                Location = new Point(20, 205),
                Size = new Size(80, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            loadTorqueTrackBar = new TrackBar
            {
                Location = new Point(100, 200),
                Size = new Size(120, 45),
                Minimum = 0,
                Maximum = 100,
                Value = (int)(SimulationConstants.DefaultLoadTorque * 1000),
                TickStyle = TickStyle.BottomRight,
                BackColor = SystemColors.Control
            };
            loadTorqueTrackBar.Scroll += (s, e) => simEngine.SetLoadTorque(loadTorqueTrackBar.Value / 1000.0);
            grpControl.Controls.Add(lblLoadTorque);
            grpControl.Controls.Add(loadTorqueTrackBar);
            Controls.Add(grpControl);

            // Fault Injection GroupBox
            GroupBox grpFaults = new GroupBox
            {
                Text = "FAULT INJECTION",
                Location = new Point(10, 370),
                Size = new Size(240, 200),
                BackColor = Color.Black,
                ForeColor = Color.White, // White title
                FlatStyle = FlatStyle.Standard
            };

            Button btnPhaseLoss = new Button
            {
                Text = "Phase Loss",
                Location = new Point(20, 20),
                Size = new Size(100, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnPhaseLoss.Click += (s, e) => faultManager.InjectPhaseLoss(0);
            grpFaults.Controls.Add(btnPhaseLoss);

            Button btnShortCircuit = new Button
            {
                Text = "Short Circuit",
                Location = new Point(20, 50),
                Size = new Size(100, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnShortCircuit.Click += (s, e) => faultManager.InjectShortCircuit(0, 1);
            grpFaults.Controls.Add(btnShortCircuit);

            Button btnSensorFailure = new Button
            {
                Text = "Sensor Failure",
                Location = new Point(20, 80),
                Size = new Size(100, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnSensorFailure.Click += (s, e) => faultManager.InjectSensorFailure();
            grpFaults.Controls.Add(btnSensorFailure);

            Button btnOvervoltage = new Button
            {
                Text = "Overvoltage",
                Location = new Point(20, 110),
                Size = new Size(100, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnOvervoltage.Click += (s, e) => faultManager.InjectOvervoltage();
            grpFaults.Controls.Add(btnOvervoltage);

            Button btnOvercurrent = new Button
            {
                Text = "Overcurrent",
                Location = new Point(20, 140),
                Size = new Size(100, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnOvercurrent.Click += (s, e) => faultManager.InjectOvercurrent();
            grpFaults.Controls.Add(btnOvercurrent);

            Button btnClearFaults = new Button
            {
                Text = "Clear Faults",
                Location = new Point(20, 170),
                Size = new Size(100, 25),
                FlatStyle = FlatStyle.Standard,
                BackColor = SystemColors.Control,
                ForeColor = Color.Black
            };
            btnClearFaults.Click += (s, e) => faultManager.ClearFaults();
            grpFaults.Controls.Add(btnClearFaults);
            Controls.Add(grpFaults);

            // Status GroupBox
            GroupBox grpStatus = new GroupBox
            {
                Text = "MOTOR STATUS",
                Location = new Point(260, 10),
                Size = new Size(240, 280),
                BackColor = Color.Black,
                ForeColor = Color.White, // White title
                FlatStyle = FlatStyle.Standard
            };

            lblSpeed = new Label
            {
                Text = "Speed: 0 RPM",
                Location = new Point(20, 20),
                Size = new Size(200, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            lblPhaseA = new Label
            {
                Text = "Phase A: 0 A",
                Location = new Point(20, 50),
                Size = new Size(200, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            lblPhaseB = new Label
            {
                Text = "Phase B: 0 A",
                Location = new Point(20, 80),
                Size = new Size(200, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            lblPhaseC = new Label
            {
                Text = "Phase C: 0 A",
                Location = new Point(20, 110),
                Size = new Size(200, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            lblIqRef = new Label
            {
                Text = "Iq Ref: 0 A",
                Location = new Point(20, 140),
                Size = new Size(200, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            lblIqAct = new Label
            {
                Text = "Iq Act: 0 A",
                Location = new Point(20, 170),
                Size = new Size(200, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            lblTorque = new Label
            {
                Text = "Torque: 0 Nm",
                Location = new Point(20, 200),
                Size = new Size(200, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            lblDCLink = new Label
            {
                Text = "DC Link: 24.00 V",
                Location = new Point(20, 230),
                Size = new Size(200, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            grpStatus.Controls.Add(lblSpeed);
            grpStatus.Controls.Add(lblPhaseA);
            grpStatus.Controls.Add(lblPhaseB);
            grpStatus.Controls.Add(lblPhaseC);
            grpStatus.Controls.Add(lblIqRef);
            grpStatus.Controls.Add(lblIqAct);
            grpStatus.Controls.Add(lblTorque);
            grpStatus.Controls.Add(lblDCLink);
            Controls.Add(grpStatus);

            // Fault Status GroupBox
            GroupBox grpFaultStatus = new GroupBox
            {
                Text = "FAULT STATUS",
                Location = new Point(260, 300),
                Size = new Size(240, 150),
                BackColor = Color.Black,
                ForeColor = Color.White, // White title
                FlatStyle = FlatStyle.Standard
            };

            lblFaultPhaseLoss = new Label
            {
                Text = "Phase Loss",
                Location = new Point(20, 20),
                Size = new Size(100, 20),
                BackColor = Color.Black,
                ForeColor = Color.White,
                BorderStyle = BorderStyle.Fixed3D
            };
            lblFaultShortCircuit = new Label
            {
                Text = "Short Circuit",
                Location = new Point(20, 50),
                Size = new Size(100, 20),
                BackColor = Color.Black,
                ForeColor = Color.White,
                BorderStyle = BorderStyle.Fixed3D
            };
            lblFaultSensor = new Label
            {
                Text = "Sensor Failure",
                Location = new Point(20, 80),
                Size = new Size(100, 20),
                BackColor = Color.Black,
                ForeColor = Color.White,
                BorderStyle = BorderStyle.Fixed3D
            };
            lblFaultOvervoltage = new Label
            {
                Text = "Overvoltage",
                Location = new Point(20, 110),
                Size = new Size(100, 20),
                BackColor = Color.Black,
                ForeColor = Color.White,
                BorderStyle = BorderStyle.Fixed3D
            };
            lblFaultOvercurrent = new Label
            {
                Text = "Overcurrent",
                Location = new Point(120, 20),
                Size = new Size(100, 20),
                BackColor = Color.Black,
                ForeColor = Color.White,
                BorderStyle = BorderStyle.Fixed3D
            };
            grpFaultStatus.Controls.Add(lblFaultPhaseLoss);
            grpFaultStatus.Controls.Add(lblFaultShortCircuit);
            grpFaultStatus.Controls.Add(lblFaultSensor);
            grpFaultStatus.Controls.Add(lblFaultOvervoltage);
            grpFaultStatus.Controls.Add(lblFaultOvercurrent);
            Controls.Add(grpFaultStatus);

            // Rotor Visualization
            GroupBox grpRotor = new GroupBox
            {
                Text = "ROTOR POSITION",
                Location = new Point(510, 10),
                Size = new Size(280, 280),
                BackColor = Color.Black,
                ForeColor = Color.White, // White title
                FlatStyle = FlatStyle.Standard
            };
            rotorPanel = new CustomPanel
            {
                Location = new Point(10, 20),
                Size = new Size(260, 240),
                BorderStyle = BorderStyle.Fixed3D,
                BackColor = Color.Black
            };
            rotorPanel.Paint += RotorPanel_Paint;
            grpRotor.Controls.Add(rotorPanel);
            Controls.Add(grpRotor);

            // Back-EMF Visualization
            GroupBox grpBemf = new GroupBox
            {
                Text = "BACK-EMF WAVEFORMS",
                Location = new Point(510, 300),
                Size = new Size(380, 250),
                BackColor = Color.Black,
                ForeColor = Color.White, // White title
                FlatStyle = FlatStyle.Standard
            };
            bemfPanel = new CustomPanel
            {
                Location = new Point(10, 20),
                Size = new Size(360, 210),
                BorderStyle = BorderStyle.Fixed3D,
                BackColor = Color.Black
            };
            bemfPanel.Paint += BemfPanel_Paint;
            grpBemf.Controls.Add(bemfPanel);
            Controls.Add(grpBemf);

            // Status Bar
            lblStatus = new Label
            {
                Text = "Status: Stopped",
                Location = new Point(10, 740),
                Size = new Size(860, 20),
                BorderStyle = BorderStyle.Fixed3D,
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            Controls.Add(lblStatus);

            // Timers
            simulationTimer = new Timer { Interval = 10 };
            simulationTimer.Tick += SimulationTimer_Tick;

            Timer uiTimer = new Timer { Interval = 100 };
            uiTimer.Tick += (s, e) =>
            {
                lblSpeed.Text = $"Speed: {motor.SpeedRpm:F0} RPM";
                var currents = motor.GetPhaseCurrents();
                lblPhaseA.Text = $"Phase A: {currents[0]:F2} A";
                lblPhaseB.Text = $"Phase B: {currents[1]:F2} A";
                lblPhaseC.Text = $"Phase C: {currents[2]:F2} A";
                var dqCurrents = motor.GetDqCurrents();
                lblIqRef.Text = $"Iq Ref: {controller.GetIqReference():F2} A";
                lblIqAct.Text = $"Iq Act: {dqCurrents[1]:F2} A";
                lblTorque.Text = $"Torque: {motor.GetElectromagneticTorque():F3} Nm";
                lblDCLink.Text = $"DC Link: {dcLink.Voltage:F2} V";
                lblFaultPhaseLoss.BackColor = faultManager.IsPhaseLossActive ? Color.Red : Color.Black;
                lblFaultShortCircuit.BackColor = faultManager.IsShortCircuitActive ? Color.Red : Color.Black;
                lblFaultSensor.BackColor = faultManager.IsSensorFailureActive ? Color.Red : Color.Black;
                lblFaultOvervoltage.BackColor = faultManager.IsOvervoltageActive ? Color.Red : Color.Black;
                lblFaultOvercurrent.BackColor = faultManager.IsOvercurrentActive ? Color.Red : Color.Black;
                string statusText = isPaused ? "Paused" : (isRunning ? "Running" : "Stopped");
                lblStatus.Text = $"Status: {statusText} | {faultManager.GetProtectionStatus()}";
            };
            uiTimer.Start();
        }

        private void BtnStartStop_Click(object sender, EventArgs e)
        {
            if (isPaused)
            {
                isPaused = false;
                ((Button)sender).Text = "Stop";
                simulationTimer.Start();
            }
            else
            {
                isRunning = !isRunning;
                ((Button)sender).Text = isRunning ? "Stop" : "Start";
                if (isRunning)
                {
                    controller.StartMotor();
                    simulationTimer.Start();
                }
                else
                {
                    simulationTimer.Stop();
                    controller.StopMotor();
                }
            }
        }

        private void BtnPause_Click(object sender, EventArgs e)
        {
            if (!isRunning) return;

            isPaused = !isPaused;
            ((Button)sender).Text = isPaused ? "Resume" : "Pause";
            if (isPaused)
            {
                simulationTimer.Stop();
            }
            else
            {
                simulationTimer.Start();
            }
        }

        private void BtnRestart_Click(object sender, EventArgs e)
        {
            isRunning = false;
            isPaused = false;
            simulationTimer.Stop();
            controller.StopMotor();

            motor.Stop();
            controller.StopMotor();
            inverter.Reset();
            dcLink.Reset();
            faultManager.ClearFaults();

            speedRefTrackBar.Value = 1000;
            loadTorqueTrackBar.Value = (int)(SimulationConstants.DefaultLoadTorque * 1000);
            cmbMotorType.SelectedIndex = 0;
            cmbPwmMode.SelectedIndex = 0;
            cmbSwitchingMode.SelectedIndex = 0;

            simEngine.SetSpeedReference(1000);
            simEngine.SetLoadTorque(SimulationConstants.DefaultLoadTorque);
            controller.SetMotorType(MotorType.Trapezoidal);
            controller.SetPwmMode(PwmMode.SixStep);
            inverter.SetSwitchingMode(SwitchingMode.Ideal);

            foreach (Control c in Controls)
            {
                if (c is GroupBox grp)
                {
                    foreach (Control btn in grp.Controls)
                    {
                        if (btn is Button startStop && btn.Text == "Stop")
                            startStop.Text = "Start";
                        if (btn is Button pause && btn.Text == "Resume")
                            pause.Text = "Pause";
                    }
                }
            }

            rotorPanel.Invalidate();
            bemfPanel.Invalidate();
        }

        private void SimulationTimer_Tick(object sender, EventArgs e)
        {
            simEngine.Update(0.01);
            // Invalidate only the shaft area to reduce flickering
            int centerX = rotorPanel.Width / 2;
            int centerY = rotorPanel.Height / 2;
            int shaftRadius = (Math.Min(rotorPanel.Width, rotorPanel.Height) / 2 - 40) / 4;
            int markerLength = (int)(shaftRadius * 1.5f);
            Rectangle shaftArea = new Rectangle(
                centerX - markerLength - 5, centerY - markerLength - 5,
                markerLength * 2 + 10, markerLength * 2 + 10);
            rotorPanel.Invalidate(shaftArea);
            bemfPanel.Invalidate();
        }

        private void RotorPanel_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.None;

            int centerX = rotorPanel.Width / 2;
            int centerY = rotorPanel.Height / 2;
            int statorRadius = Math.Min(rotorPanel.Width, rotorPanel.Height) / 2 - 20;
            int rotorRadius = statorRadius - 20;
            int shaftRadius = rotorRadius / 4;
            float rotorAngleRad = (float)(motor.RotorAngle * Math.PI / 180.0);

            // Draw stator
            using (Pen statorPen = new Pen(Color.DarkGray, 8))
            {
                g.DrawEllipse(statorPen, centerX - statorRadius, centerY - statorRadius, statorRadius * 2, statorRadius * 2);
                for (int i = 0; i < 12; i++)
                {
                    float slotAngle = (float)(i * 30.0 * Math.PI / 180.0);
                    int slotOuterX = (int)(centerX + statorRadius * Math.Cos(slotAngle));
                    int slotOuterY = (int)(centerY + statorRadius * Math.Sin(slotAngle));
                    int slotInnerX = (int)(centerX + (statorRadius - 5) * Math.Cos(slotAngle));
                    int slotInnerY = (int)(centerY + (statorRadius - 5) * Math.Sin(slotAngle));
                    g.DrawLine(new Pen(Color.DarkGray, 2), slotOuterX, slotOuterY, slotInnerX, slotInnerY);
                }
            }

            // Draw rotor
            Rectangle rotorRect = new Rectangle(centerX - rotorRadius, centerY - rotorRadius, rotorRadius * 2, rotorRadius * 2);
            using (SolidBrush rotorBrush = new SolidBrush(Color.Silver))
            {
                g.FillEllipse(rotorBrush, rotorRect);
            }

            // Draw magnetic poles
            int poleCount = 4;
            float poleAngle = 360.0f / poleCount;
            for (int i = 0; i < poleCount; i++)
            {
                float startAngle = i * poleAngle;
                Color poleColor = (i % 2 == 0) ? Color.Red : Color.Blue;
                using (SolidBrush poleBrush = new SolidBrush(poleColor))
                using (GraphicsPath polePath = new GraphicsPath())
                {
                    polePath.AddArc(centerX - rotorRadius, centerY - rotorRadius, rotorRadius * 2, rotorRadius * 2,
                        startAngle, poleAngle);
                    polePath.AddArc(centerX - shaftRadius, centerY - shaftRadius, shaftRadius * 2, shaftRadius * 2,
                        startAngle + poleAngle, -poleAngle);
                    polePath.CloseFigure();
                    g.FillPath(poleBrush, polePath);
                }
            }

            // Draw shaft
            Rectangle shaftRect = new Rectangle(centerX - shaftRadius, centerY - shaftRadius, shaftRadius * 2, shaftRadius * 2);
            using (SolidBrush shaftBrush = new SolidBrush(Color.Black))
            {
                g.FillEllipse(shaftBrush, shaftRect);
            }

            // Draw needle
            float markerLength = shaftRadius * 1.5f;
            int markerX = (int)(centerX + markerLength * Math.Cos(rotorAngleRad));
            int markerY = (int)(centerY + markerLength * Math.Sin(rotorAngleRad));
            g.DrawLine(new Pen(Color.Yellow, 3), centerX, centerY, markerX, markerY);
        }

        private void BemfPanel_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.None;

            int width = bemfPanel.Width - 40;
            int height = bemfPanel.Height - 40;
            int centerY = bemfPanel.Height / 2;

            // Draw grid
            for (int y = centerY - height / 2; y <= centerY + height / 2; y += height / 8)
                g.DrawLine(new Pen(Color.DarkGray), 20, y, width + 20, y);
            for (int x = 20; x <= width + 20; x += width / 8)
                g.DrawLine(new Pen(Color.DarkGray), x, 20, x, height + 20);

            // Draw axes
            g.DrawLine(new Pen(Color.White, 2), 20, centerY, width + 20, centerY);
            g.DrawLine(new Pen(Color.White, 2), 20, 20, 20, height + 20);

            // Draw waveforms
            PointF[] pointsA = new PointF[width];
            PointF[] pointsB = new PointF[width];
            PointF[] pointsC = new PointF[width];
            for (int x = 0; x < width; x++)
            {
                float t = (float)x / width * 360.0f;
                float scale = height / 4.0f;
                pointsA[x] = new PointF(x + 20, centerY - (float)motor.GetBackEMFPhase(t, 0) * scale);
                pointsB[x] = new PointF(x + 20, centerY - (float)motor.GetBackEMFPhase(t - 120, 1) * scale);
                pointsC[x] = new PointF(x + 20, centerY - (float)motor.GetBackEMFPhase(t - 240, 2) * scale);
            }

            g.DrawLines(new Pen(Color.Red, 2), pointsA);
            g.DrawLines(new Pen(Color.Green, 2), pointsB);
            g.DrawLines(new Pen(Color.Blue, 2), pointsC);
        }
    }
}