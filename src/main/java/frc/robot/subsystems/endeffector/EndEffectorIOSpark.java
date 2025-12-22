package frc.robot.subsystems.endeffector;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class EndEffectorIOSpark implements EndEffectorIO {
  private final SparkBase rollerSpark;
  // private final Queue<Double> timestampQueue;
  // private final Queue<Double> holdingCoralQueue;
  // private DigitalInput frontIntakeBeamBreak;
  // private DigitalInput backIntakeBeamBreak;
  // private final StatusSignal<boolean> frontBBBlocked= frontIntakeBeamBreak.get();
  private final Debouncer endEffectorConnectedDebounce = new Debouncer(0.5);

  public EndEffectorIOSpark() {
    rollerSpark = new SparkMax(endEffectorCANID, MotorType.kBrushless);
    var endEffectorConfig = new SparkMaxConfig();
    endEffectorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(endEffectorCurrentLimit)
        .voltageCompensation(12.0);
    endEffectorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        rollerSpark,
        5,
        () ->
            rollerSpark.configure(
                endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    // Update endEffector inputs
    sparkStickyFault = false;
    ifOk(
        rollerSpark,
        new DoubleSupplier[] {rollerSpark::getAppliedOutput, rollerSpark::getBusVoltage},
        (values) -> inputs.endEffectorAppliedVolts = values[0] * values[1]);
    ifOk(
        rollerSpark,
        rollerSpark::getOutputCurrent,
        (value) -> inputs.endEffectorCurrentAmps = value);
    inputs.endEffectorConnected = endEffectorConnectedDebounce.calculate(!sparkStickyFault);

    // inputs.endEffectorTimestamps =
    //     timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.holdingCoral =
    //     holdingCoralQueue.stream().mapToDouble((Double value) -> value).toArray();

  }

  @Override
  public void setEndEffectorOpenLoop(double output) {
    rollerSpark.setVoltage(output);
  }
}
