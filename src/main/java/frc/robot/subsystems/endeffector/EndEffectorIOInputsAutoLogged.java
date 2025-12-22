package frc.robot.subsystems.endeffector;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class EndEffectorIOInputsAutoLogged extends EndEffectorIO.EndEffectorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("endEffectorConnected", endEffectorConnected);
    table.put("endEffectorAppliedVolts", endEffectorAppliedVolts);
    table.put("endEffectorCurrentAmps", endEffectorCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    endEffectorConnected = table.get("endEffectorConnected", endEffectorConnected);
    endEffectorAppliedVolts = table.get("endEffectorAppliedVolts", endEffectorAppliedVolts);
    endEffectorCurrentAmps = table.get("endEffectorCurrentAmps", endEffectorCurrentAmps);
  }

  public EndEffectorIOInputsAutoLogged clone() {
    EndEffectorIOInputsAutoLogged copy = new EndEffectorIOInputsAutoLogged();
    copy.endEffectorConnected = this.endEffectorConnected;
    copy.endEffectorAppliedVolts = this.endEffectorAppliedVolts;
    copy.endEffectorCurrentAmps = this.endEffectorCurrentAmps;
    return copy;
  }
}
