package frc.robot.subsystems.endeffector;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {
  private final DCMotorSim rollerSim;
  private double EndEffectorAppliedVolts = 0.0;

  public EndEffectorIOSim() {
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rollerGearbox, 0.004, rollerMotorReduction),
            rollerGearbox);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    rollerSim.setInputVoltage(MathUtil.clamp(EndEffectorAppliedVolts, -12.0, 12.0));
    rollerSim.update(0.02);
    inputs.EndEffectorConnected = true;
    inputs.EndEffectorAppliedVolts = EndEffectorAppliedVolts;
    inputs.EndEffectorCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
  }

  @Override
  public void setEndEffectorOpenLoop(double output) {
    EndEffectorAppliedVolts = output;
  }
}
