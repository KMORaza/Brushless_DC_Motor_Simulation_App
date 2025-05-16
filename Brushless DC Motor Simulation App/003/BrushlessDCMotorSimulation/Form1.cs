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
        private CheckBox chkMtpa, chkFieldWeakening;
        private bool isRunning;
        private bool isPaused;

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
            Text = "Brushless DC Motor Driver";
            Size = new Size(910, 800);
            BackColor = Color.Black;
            Font = new Font("Consolas", 11f);
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;

            // Control GroupBox
            GroupBox grpControl = new GroupBox
            {
                Text = "Motor Control",
                Location = new Point(10, 10),
                Size = new Size(240, 400),
                BackColor = Color.Black,
                ForeColor = Color.White,
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
                Location = new Point(20, 200),
                Size = new Size(80, 20),
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            loadTorqueTrackBar = new TrackBar
            {
                Location = new Point(100, 195),
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

            chkMtpa = new CheckBox
            {
                Text = "Enable MTPA",
                Location = new Point(20, 250),
                Size = new Size(100, 20),
                Checked = true,
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            chkMtpa.CheckedChanged += (s, e) => controller.SetMtpaEnabled(chkMtpa.Checked);
            grpControl.Controls.Add(chkMtpa);

            chkFieldWeakening = new CheckBox
            {
                Text = "Enable Field Weakening",
                Location = new Point(20, 280),
                Size = new Size(150, 20),
                Checked = true,
                BackColor = Color.Black,
                ForeColor = Color.White
            };
            chkFieldWeakening.CheckedChanged += (s, e) => controller.SetFieldWeakeningEnabled(chkFieldWeakening.Checked);
            grpControl.Controls.Add(chkFieldWeakening);

            Controls.Add(grpControl);

            // Fault Injection GroupBox
            GroupBox grpFaults = new GroupBox
            {
                Text = "Fault Injection",
                Location = new Point(10, 420),
                Size = new Size(240, 200),
                BackColor = Color.Black,
                ForeColor = Color.White,
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
                Text = "Motor Status",
                Location = new Point(260, 10),
                Size = new Size(240, 280),
                BackColor = Color.Black,
                ForeColor = Color.White,
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
                Text = "Faults",
                Location = new Point(260, 300),
                Size = new Size(240, 150),
                BackColor = Color.Black,
                ForeColor = Color.White,
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
                ForeColor = Color.White,
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
                Text = "Waveform",
                Location = new Point(510, 300),
                Size = new Size(380, 250),
                BackColor = Color.Black,
                ForeColor = Color.White,
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
            chkMtpa.Checked = true;
            chkFieldWeakening.Checked = true;

            simEngine.SetSpeedReference(1000);
            simEngine.SetLoadTorque(SimulationConstants.DefaultLoadTorque);
            controller.SetMotorType(MotorType.Trapezoidal);
            controller.SetPwmMode(PwmMode.SixStep);
            inverter.SetSwitchingMode(SwitchingMode.Ideal);
            controller.SetMtpaEnabled(true);
            controller.SetFieldWeakeningEnabled(true);

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

            Rectangle rotorRect = new Rectangle(centerX - rotorRadius, centerY - rotorRadius, rotorRadius * 2, rotorRadius * 2);
            using (SolidBrush rotorBrush = new SolidBrush(Color.Silver))
            {
                g.FillEllipse(rotorBrush, rotorRect);
            }

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

            Rectangle shaftRect = new Rectangle(centerX - shaftRadius, centerY - shaftRadius, shaftRadius * 2, shaftRadius * 2);
            using (SolidBrush shaftBrush = new SolidBrush(Color.Black))
            {
                g.FillEllipse(shaftBrush, shaftRect);
            }

            float markerLength = shaftRadius * 1.5f;
            int markerX = (int)(centerX + markerLength * Math.Cos(rotorAngleRad));
            int markerY = (int)(centerY + markerLength * Math.Sin(rotorAngleRad));
            g.DrawLine(new Pen(Color.Yellow, 3), centerX, centerY, markerX, markerY);
        }

        private void BemfPanel_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias; // Enable anti-aliasing for smoother lines

            int width = bemfPanel.Width - 60; // Extra margin for annotations
            int height = bemfPanel.Height - 60;
            int centerY = bemfPanel.Height / 2;
            int marginX = 40;
            int marginY = 30;

            // Calculate max back-EMF based on motor speed
            double omega = motor.SpeedRpm * 2 * Math.PI / 60.0; // rad/s
            double maxBackEmf = SimulationConstants.BackEmfConstant * omega; // Volts
            maxBackEmf = Math.Max(0.1, maxBackEmf); // Avoid zero scaling

            // Get current rotor angle for phase offset
            float rotorAngle = (float)(motor.RotorAngle % 360.0); // Current electrical angle in degrees

            // Draw finer grid
            using (Pen gridPen = new Pen(Color.DarkGray, 1))
            {
                // Horizontal lines (voltage)
                for (int y = centerY - height / 2; y <= centerY + height / 2; y += height / 16) // Finer grid
                {
                    g.DrawLine(gridPen, marginX, y, width + marginX, y);
                }
                // Vertical lines (time/angle)
                for (int x = marginX; x <= width + marginX; x += width / 16)
                {
                    g.DrawLine(gridPen, x, marginY, x, height + marginY);
                }
            }

            // Draw axes
            using (Pen axisPen = new Pen(Color.White, 2))
            {
                g.DrawLine(axisPen, marginX, centerY, width + marginX, centerY); // X-axis (0V)
                g.DrawLine(axisPen, marginX, marginY, marginX, height + marginY); // Y-axis
            }

            // Draw axis labels
            using (Font labelFont = new Font("Arial", 8))
            using (SolidBrush labelBrush = new SolidBrush(Color.White))
            {
                // Voltage labels (Y-axis)
                g.DrawString($"{maxBackEmf:F1} V", labelFont, labelBrush, marginX - 30, marginY - 10);
                g.DrawString("0 V", labelFont, labelBrush, marginX - 20, centerY - 10);
                g.DrawString($"-{maxBackEmf:F1} V", labelFont, labelBrush, marginX - 30, height + marginY - 10);
                // Time/angle label (X-axis)
                g.DrawString("0°", labelFont, labelBrush, marginX - 10, height + marginY + 5);
                g.DrawString("360°", labelFont, labelBrush, width + marginX - 20, height + marginY + 5);
            }

            // Plot back-EMF waveforms with rotor angle offset
            PointF[] pointsA = new PointF[width];
            PointF[] pointsB = new PointF[width];
            PointF[] pointsC = new PointF[width];
            for (int x = 0; x < width; x++)
            {
                float t = (float)x / width * 360.0f + rotorAngle; // Angle in degrees, offset by rotor angle
                float scale = (float)(height / (2.0 * maxBackEmf)); // Dynamic scaling
                pointsA[x] = new PointF(x + marginX, centerY - (float)motor.GetBackEMFPhase(t, 0) * (float)maxBackEmf * scale);
                pointsB[x] = new PointF(x + marginX, centerY - (float)motor.GetBackEMFPhase(t - 120, 1) * (float)maxBackEmf * scale);
                pointsC[x] = new PointF(x + marginX, centerY - (float)motor.GetBackEMFPhase(t - 240, 2) * (float)maxBackEmf * scale);
            }

            // Draw waveforms with thicker lines for clarity
            using (Pen penA = new Pen(Color.Red, 2))
            using (Pen penB = new Pen(Color.Green, 2))
            using (Pen penC = new Pen(Color.Blue, 2))
            {
                g.DrawLines(penA, pointsA);
                g.DrawLines(penB, pointsB);
                g.DrawLines(penC, pointsC);
            }

            // Draw legend
            using (Font legendFont = new Font("Arial", 8))
            using (SolidBrush legendBrush = new SolidBrush(Color.White))
            {
                g.DrawString("Phase A", legendFont, legendBrush, marginX + 10, marginY + 10);
                g.DrawLine(new Pen(Color.Red, 2), marginX + 60, marginY + 15, marginX + 80, marginY + 15);
                g.DrawString("Phase B", legendFont, legendBrush, marginX + 10, marginY + 25);
                g.DrawLine(new Pen(Color.Green, 2), marginX + 60, marginY + 30, marginX + 80, marginY + 30);
                g.DrawString("Phase C", legendFont, legendBrush, marginX + 10, marginY + 40);
                g.DrawLine(new Pen(Color.Blue, 2), marginX + 60, marginY + 45, marginX + 80, marginY + 45);
            }
        }
    }
}