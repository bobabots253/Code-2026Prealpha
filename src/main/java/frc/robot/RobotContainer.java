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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  /*
   * Where is the file path for the 'swerveSubsystem'? Where is the file path for the 'vision'?
   * 
   * - File path for the swerve subsystem is src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java.
   * - File path for the vision folder is src/main/java/frc/robot/subsystems/vision.
   */
  private final SwerveSubsystem
      swerveSubsystem; // creates a new swerve subsystem variable to be initialized
  private final Vision
      vision; // creates a new vision variable to be initialized /* What type of variable? - An instance of the Vision Class.*/

  // Controller
  private final CommandXboxController controller =
      new CommandXboxController(
          0); // creates a new xbox controller object; this variable will refer to the controller
  // connected to port 0?

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // I'm not sure what this chooser thing is, but I think this would be related to the smart
  // dashboard judging by its class name
  /* Nice, this LoggedDashboardChooser extends of the WPILib 'SendableChooser' class which we use to select out autos */
  /* The only difference between this one and the default WPILib is one is that it makes it loggable - got it*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { // constructor
    switch (Constants
        .currentMode) { // switch statement to determine how to initialize swerve and vision
      case REAL: // if the program is running on an actual physical robot
        // Real robot, instantiate hardware IO implementations
        swerveSubsystem =
            new SwerveSubsystem( // initialize variable to a new swerve subsystem object with new
                // motor objects
                new GyroIOPigeon2(), // idk /* The GyroIO is our gyroscope we use to calculate wheel
                // odometry, the name of the specific product is called a
                // Pigeon 2.0 - got it*/
                new ModuleIOSpark(
                    0), // front left motor /* Position Correct, What type of motor? - I think it's a combination of two motors, Spark Flex and Spark Max?*/
                new ModuleIOSpark(
                    1), // front right motor /* Position Correct, What type of motor?*/
                new ModuleIOSpark(2), // back left motor /* Position Correct, What type of motor?*/
                new ModuleIOSpark(
                    3)); // back right motor /* Position Correct, What type of motor?*/

        vision =
            new Vision( // initialize variable to a new vision object with new limelight objects
                swerveSubsystem
                    ::addVisionMeasurement, // adds a timestamped vision measurement, i'm not sure
                // what that means
                /* A timestamp is a digital marker that allows us to sort the collected data */
                /* If both camera return a vision input (along with a timestamp), we know the camera frames are synced up - understood*/
                new VisionIOLimelight(
                    VisionConstants.FrontLeftLL,
                    swerveSubsystem
                        ::getRotation), // create new limelight camera objects with specified IDs
                // and configs
                new VisionIOLimelight(
                    VisionConstants.FrontRightLL,
                    swerveSubsystem::getRotation)); /* What does the getRotation do? - Return the odometric position and orientation of the robot*/

        break;

        // I'm unsure how simulation works.
      case SIM: // if the program is running on a simulator
        // Sim robot, instantiate physics sim IO implementations
        swerveSubsystem = // initialization
            new SwerveSubsystem(
                new GyroIO() {}, /* In simulation, no real hardware can be instantiated so we use the inbuilt WPILib Gyro class - got it*/
                new ModuleIOSim(), // simulated motor modules /* Good */
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision = // initialization
            new Vision(
                swerveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVisionSim( // simulator specific configurations?
                    /* Yes! It specificies the translation of the simulated cameras relative to the center of the robot. */
                    VisionConstants.FrontLeftLL,
                    VisionConstants.robotToFrontLeftLL,
                    swerveSubsystem::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.FrontRightLL,
                    VisionConstants.robotToFrontRightLL,
                    swerveSubsystem::getPose));
        break;

      default: // if the program is running on a replay log file (not 100% sure what that means)
        /* In replay mode, the robot uses the recorded values, there is no need to recalculate each of the value, we are just "watching" - got it*/
        // Replayed robot, disable IO implementations
        swerveSubsystem =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(swerveSubsystem::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Choices",
            AutoBuilder
                .buildAutoChooser()); // does this determine the movement for the autonomous period?
    // /* Yes! */

    // Set up SysId routines
    autoChooser.addOption( // auto options? I'm confused on what these options do.
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(swerveSubsystem));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(swerveSubsystem));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        swerveSubsystem.sysIdDynamic(
            SysIdRoutine.Direction.kReverse)); // <-- Goofy Compile Time Syntax  Error

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() { // bind button controls to the xbox controller
    // Default command, normal field-relative drive
    swerveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive( // bind the joystick controls to the swerve subsystem
            swerveSubsystem,
            () -> -controller.getLeftY(), // left joystick, vertical tilt
            () -> -controller.getLeftX(), // left joystick, horizontal tilt
            () -> -controller.getRightX())); // right joystick, horizontal tilt

    // Lock to 0° when A button is held
    controller // bind A button control to swerve
        .a()
        .whileTrue( // while the button is pressed down
            DriveCommands.joystickDriveAtAngle(
                swerveSubsystem,
                () -> -controller.getLeftY(), // left joystick vertical
                () -> -controller.getLeftX(), // left joystick horizontal
                () -> new Rotation2d())); // constructor for measure of rotation?
    /* Not Quite, when a blank Rotation2D is made, the angle is defaulted to 0* - understood*/

    // Switch to X pattern when X button is pressed
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                swerveSubsystem::stopWithX,
                swerveSubsystem)); // bind X button control to swerve, not sure what X pattern is

    // Reset gyro to 0° when B button is pressed
    controller // bind B button control to swerve
        .b()
        .onTrue( // while B button is pressed
            Commands.runOnce(
                    () ->
                        swerveSubsystem.setPose( // is pose the measure of translation & rotation?
                            new Pose2d(
                                swerveSubsystem.getPose().getTranslation(), new Rotation2d())),
                    swerveSubsystem)
                .ignoringDisable(true)); // makes it so this command runs when disabled? /* Yes */

    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // controller
    //     .y()
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //                 swerveSubsystem.run(0.0,
    // aimController.calculate(vision.getTargetX(0).getRadians()));
    //             },
    //             swerveSubsystem));
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { // get what command to run during auto
    return autoChooser.get(); // chooses the command to use during auto?
  }
}
