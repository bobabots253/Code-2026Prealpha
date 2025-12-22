package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public boolean endEffectorConnected = false;
    public double endEffectorAppliedVolts = 0.0;
    public double endEffectorCurrentAmps = 0.0;

    public double[] endEffectorTimestamps = new double[] {};
    public double[] holdingCoral = new double[] {};
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEndEffectorOpenLoop(double output) {}
}
