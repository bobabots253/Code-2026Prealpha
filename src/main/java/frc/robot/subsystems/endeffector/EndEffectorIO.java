package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public boolean EndEffectorConnected = false;
    public double EndEffectorAppliedVolts = 0.0;
    public double EndEffectorCurrentAmps = 0.0;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEndEffectorOpenLoop(double output) {}
}
