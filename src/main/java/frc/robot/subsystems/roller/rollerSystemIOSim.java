// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.roller;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class rollerSystemIOSim implements rollerSystemIO {
  private final DCMotorSim rollerSim;

  private double AppliedVolts = 0.0;

  public rollerSystemIOSim() {
    // Create roller sim models
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 1, 0.001),
            DCMotor.getNeoVortex(1));

    // Enable wrapping for turn PID
    // turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(rollerSystemIOInputs inputs) {
    // Run closed-loop control

    if (DriverStation.isDisabled()) {
      setOpenLoop(0.0);
    }

    // Update simulation state

    rollerSim.update(0.02);

    // Update drive inputs

    inputs.positionRads = rollerSim.getAngularPositionRad();
    inputs.velocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = AppliedVolts;
    // inputs.appliedVoltage = rollerSim.getInputVoltage();
    inputs.currentAmps = Math.abs(rollerSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
  }

  @Override
  public void setOpenLoop(double output) {
    AppliedVolts = output;
    rollerSim.setInputVoltage(MathUtil.clamp(AppliedVolts, -12.0, 12.0));
  }

  @Override
  public void stop() {
    setOpenLoop(0.0);
  }
}
