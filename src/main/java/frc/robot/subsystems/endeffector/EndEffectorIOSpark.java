package frc.robot.subsystems.endeffector;
import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
//import edu.wpi.first.wpilibj.DigitalInput;
public class EndEffectorIOSpark implements EndEffectorIO{
    private final SparkBase rollerSpark;
    //private final DigitalInput frontIntakeBeamBreak;
    //private final DigitalInput backIntakeBeamBreak;

  private final Debouncer endEffectorConnectedDebounce = new Debouncer(0.5);

    public EndEffectorIOSpark(){
    //frontIntakeBeamBreak = new DigitalInput(frontBeamBreakSensor);
    //backIntakeBeamBreak = new DigitalInput(backBeamBreakSensor);
    rollerSpark = new SparkMax(endEffectorCanID,MotorType.kBrushless);
    var rollerConfig = new SparkMaxConfig();
    rollerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(endEffectorCurrentLimit)
        .voltageCompensation(12.0);
    rollerConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        rollerSpark,
        5,
        () ->
            rollerSpark.configure(
                rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));


    
}
@Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(
        rollerSpark,
        new DoubleSupplier[] {rollerSpark::getAppliedOutput, rollerSpark::getBusVoltage},
        (values) -> inputs.EndEffectorAppliedVolts = values[0] * values[1]);
    ifOk(rollerSpark, rollerSpark::getOutputCurrent, (value) -> inputs.EndEffectorCurrentAmps = value);
    inputs.EndEffectorConnected = endEffectorConnectedDebounce.calculate(!sparkStickyFault);
    
  }
  @Override
  public void setEndEffectorOpenLoop(double output) {
    rollerSpark.setVoltage(output);
  }


} 