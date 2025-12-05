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

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs { //creates class for updating Gyro in simulations and replays
    public boolean connected = false; //declares placeholder parameters to be implemented in the GyroPigeon2 file 
    public Rotation2d yawPosition = new Rotation2d(); // create a rotation in degress
    public double yawVelocityRadPerSec = 0.0; //turn rate in rad per second
    public double[] odometryYawTimestamps = new double[] {}; //array of timestamps for rotations
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {}; //array of Rotations for Gyro
  }

  public default void updateInputs(GyroIOInputs inputs) {} //method template that is overwritten in Pigeon2 file
}
