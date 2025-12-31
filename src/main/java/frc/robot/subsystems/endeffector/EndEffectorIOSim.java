package frc.robot.subsystems.endeffector;
import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.ModuleIO.ModuleIOInputs;

public class EndEffectorIOSim implements EndEffectorIO {
    private final DCMotorSim rollerSim;
    private double EndEffectorAppliedVolts = 0.0;

    public EndEffectorIOSim (){
        rollerSim = 
            new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerGearbox, 0.004, rollerMotorReduction), rollerGearbox);
    }
    @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    // Update simulation state
    rollerSim.setInputVoltage(MathUtil.clamp(EndEffectorAppliedVolts, -12.0, 12.0));
    rollerSim.update(0.02);

    // Update drive inputs
    inputs.EndEffectorConnected = true;
    inputs.EndEffectorAppliedVolts = EndEffectorAppliedVolts;
    inputs.EndEffectorCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
  }
  @Override
  public void setEndEffectorOpenLoop(double output) {
    EndEffectorAppliedVolts = output;
  }

  
}