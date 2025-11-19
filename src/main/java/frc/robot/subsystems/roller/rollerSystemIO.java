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

import org.littletonrobotics.junction.AutoLog;

public interface rollerSystemIO {
  @AutoLog
  public static class rollerSystemIOInputs {
    public double positionRads;
    public double velocityRadPerSec;
    public double appliedVoltage;
    public double currentAmps;
    public boolean connected;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(rollerSystemIOInputs inputs) {}

  /** Run the roller motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Stops motor (set speed to 0) */
  public default void stop() {}

  /** set the max current the motor can go */
  public default void setCurrentLimit(double currentLimit) {}

  /** set the brake mode of the roller motor */
  public default void setBrakeMode(boolean enabled) {}
}
