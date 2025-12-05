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

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO { //implements placeholders from GYROIO
  private final Pigeon2 pigeon = new Pigeon2(pigeonCanId); //initializes the pigeon (gyro) in code
  private final StatusSignal<Angle> yaw = pigeon.getYaw(); //gets live data for yaw of the pigeon
  private final Queue<Double> yawPositionQueue; //initializes the queues for the yaw timestamp and position arraylists for the SparkOdometryThread
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();//gets data for the angular velocity, from z axis, relative to the world.

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration()); //applies a new configuration
    pigeon.getConfigurator().setYaw(0.0); // zeroes the yaw
    yaw.setUpdateFrequency(odometryFrequency); //sets the frequency at which we publish the yaw StatusSignal
    yawVelocity.setUpdateFrequency(50.0); //same thing but for turn rate
    pigeon.optimizeBusUtilization(); //optimizes use of CanBus for the pigeon, by delaying signals update frequency, and disabling certain signals
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue(); // creates a timestamp queue from the OdometryThread
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble); //creates a queue that registers the yaw signal, converting the value to a double.
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) { // method that overrides the old one in the abstract method GyroIO
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK); //tells the code the pigeon is connected if there are no errors inyaw or yawvelocity signals
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble()); //sets the yaw position to the yaw value as a double
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble()); //same but with yaw velocity/ turn rate

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray(); // puts the queue of timestamps, as doubles, into an array
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new); //creates an array of new Rotations.
    yawTimestampQueue.clear(); //removes all the data from the two queues
    yawPositionQueue.clear();
  }
}
