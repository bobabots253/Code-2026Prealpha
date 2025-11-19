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
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class rollerSystemIOSpark implements rollerSystemIO {
  // Hardware objects
  private final SparkBase rollerSpark;
  private final RelativeEncoder rollerEncoder;
  private final SparkBaseConfig config;

  // Connection debouncers
  private int currentLimit = 60;
  // private boolean brakeModeEnabled = true;

  public rollerSystemIOSpark(int id) {
    rollerSpark = new SparkMax(id, MotorType.kBrushless);
    rollerEncoder = rollerSpark.getEncoder();

    // Configure drive motor
    config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        rollerSpark,
        5,
        () ->
            rollerSpark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(rollerSpark, 5, () -> rollerEncoder.setPosition(0.0));

    // Configure turn motor

    // Create odometry queues
  }

  @Override
  public void updateInputs(rollerSystemIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(rollerSpark, rollerEncoder::getPosition, (value) -> inputs.positionRads = value);
    ifOk(rollerSpark, rollerEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        rollerSpark,
        new DoubleSupplier[] {rollerSpark::getAppliedOutput, rollerSpark::getBusVoltage},
        (values) -> inputs.appliedVoltage = values[0] * values[1]);
    inputs.connected = false;

    // Update odometry inputs
  }

  @Override
  public void setOpenLoop(double output) {
    rollerSpark.setVoltage(output);
  }

  @Override
  public void stop() {
    setOpenLoop(0.0);
  }
}
