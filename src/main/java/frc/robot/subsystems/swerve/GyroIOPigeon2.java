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
// Uses the empty method from GyroIO interface
public class GyroIOPigeon2 implements GyroIO {
  //define a new pigeon gyro
  private final Pigeon2 pigeon = new Pigeon2(pigeonCanId);
  //Creates a Status signal which will be used to retrive the robot's yaw
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  //creates Queue to store data for later use (processing)
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  //Creates a Status signal which will be used to retrive the robot's angular velocity
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    //apply configs (basically settings) to the Pigeon, just like we do with sparkMaxes
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    //Zero the Yaw
    pigeon.getConfigurator().setYaw(0.0);
    //Update Speed for yaw / angular velocity data from the robot (50 Hz)
    yaw.setUpdateFrequency(odometryFrequency);
    yawVelocity.setUpdateFrequency(50.0);
    //Disables all status signals not explicitly given an update frequency (does this do anything in current code if both status signals have an update freq?)
    pigeon.optimizeBusUtilization();
    //Creates a queue for timestamp data (that can be fed into the odometryYawTimestamps array later) using the SparkOdemetryThread file
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    //Creates a queue for yaw position data (that can be fed into the odometryYawTimestamps array later) using the SparkOdemetryThread file, which is returned as a double (what does '::' mean?)
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
  }

  @Override
  //uses blank method from GyroIO interface
  public void updateInputs(GyroIOInputs inputs) {
    //returns 'connected' as true if yaw and yawvelocity StatusSignals are able to be refreshed by the Gyro with OK signals
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    //get the current yaw value
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    //get the current angular velocity
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    //puts the data stored in the Queues into arrays
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    //clears the queues of all data. This will keep code processing optmized since the queues no longer need to store data that has been moved to arrays
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

// IO Theory
//1. Create an interface with empty methods
//2. Create a specific class defining the interface
//3. Have that class collect data (inputs) and use that data for actions (outputs)

//Pigeon Creation Basics
//1. Create / Define attributes [StatusSignals, Queues (Only intialized), The Piegon Gyro itself]
//2. Configure the Gyro as well as Status Signals, define queues
//3. Use defined Status Signals to collect data -> Store it in queues -> put it into arrays for more permanent storge, clear queues after