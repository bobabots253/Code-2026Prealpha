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

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs { // list of possible inputs for a swerve module
    public boolean driveConnected =
        false; // whether or not drive motor is connected (used for alerts if false)
    public double drivePositionRad = 0.0; // position of a drive motor in radians
    public double driveVelocityRadPerSec =
        0.0; // how many radians a module's wheel rotates in one second
    public double driveAppliedVolts = 0.0; // how may volts are given to the module
    public double driveCurrentAmps = 0.0; // how many amps the module is drawing (SIM)

    public boolean turnConnected =
        false; // whether or not turning motor is connected (used for alerts if false)
    public Rotation2d turnPosition = new Rotation2d(); // turning motor's rotation
    public double turnVelocityRadPerSec =
        0.0; // same stuff as above but for turning motot instead of driving one.
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}
}
