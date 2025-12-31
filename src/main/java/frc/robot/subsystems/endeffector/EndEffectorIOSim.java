package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;

public class EndEffectorIOSim implements EndEffectorIO {
    private final DCMotorSim rollerSim;
    private double endEffectorAppliedVolts = 0.0;
public EndEffectorIOSim () {
    rollerSim = new DCMotorSim (
        LinearSystemId.createDCMotorSystem(endEffectorGearbox, 0.004, endEffectorMotorReduction),
        endEffectorGearbox);
}
@Override
  public void updateInputs(EndEffectorIOInputs inputs) {
rollerSim.setInputVoltage(MathUtil.clamp(endEffectorAppliedVolts, -12.0, 12.0));
rollerSim.update(0.02);
inputs.endEffectorConnected = true;
inputs.endEffectorAppliedVolts = endEffectorAppliedVolts;
inputs.endEffectorCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
  }
@Override
public void setEndEffectorOpenLoop(double output) {
  endEffectorAppliedVolts = output;
}
}

