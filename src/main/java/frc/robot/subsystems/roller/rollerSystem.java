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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class rollerSystem extends SubsystemBase {
  static final Lock threadLock = new ReentrantLock();
  protected final rollerSystemIOInputsAutoLogged inputs = new rollerSystemIOInputsAutoLogged();

  private final rollerSystemIO io;
  private final Alert disconnected;
  private boolean hasGamePiece = false;
  private static final double ROLLER_HEIGHT = 0.2; // 0.2m off ground
  private static final double ROLLER_X = 0.4;

  public rollerSystem(rollerSystemIO io) {
    this.io = io;

    disconnected = new Alert("Roller motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("rollerSystem", inputs);

    Pose3d rollerPose =
        new Pose3d(
            ROLLER_X,
            0.0,
            ROLLER_HEIGHT,
            new Rotation3d(0, inputs.positionRads, 0) // Rotates around Y axis
            );
    Logger.recordOutput("rollerSystem/Pose3d", rollerPose);

    // Detect game piece (when current spikes and roller stalls)
    if (inputs.currentAmps > 20 && Math.abs(inputs.velocityRadPerSec) < 1.0) {
      hasGamePiece = true;
    }

    // Log game piece position if we have it
    if (hasGamePiece) {
      Pose3d gamePiecePose = new Pose3d(ROLLER_X, 0.0, ROLLER_HEIGHT, new Rotation3d());
      Logger.recordOutput("GamePieces/Held", new Pose3d[] {gamePiecePose});
    } else {
      Logger.recordOutput("GamePieces/Held", new Pose3d[] {});
    }
  }

  // @AutoLogOutput
  public void setSpeed(double speed) {
    io.setOpenLoop(speed);
  }

  public void outtake() {
    hasGamePiece = false;
    setSpeed(-.5);
  }
}
