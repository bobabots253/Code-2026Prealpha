package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;

public class EndEffectorConstants {
  public static final int endEffectorCanID = 99;
  public static final int endEffectorCurrentLimit = 25;
  public static final int frontBeamBreakSensor = 1;
  public static final int backBeamBreakSensor = 2;
  
  public static final DCMotor rollerGearbox = DCMotor.getNeo550(1);
  public static final double rollerMotorReduction = 9424.0 / 203.0; //not sure if this is correct, I just took the turn motor reduction since its also a neo 550

}
