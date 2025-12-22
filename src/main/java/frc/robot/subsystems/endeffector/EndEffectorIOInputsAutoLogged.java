package frc.robot.subsystems.endeffector;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class EndEffectorIOInputsAutoLogged extends EndEffectorIO.EndEffectorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("EndEffectorConnected", EndEffectorConnected);
    table.put("EndEffectorAppliedVolts", EndEffectorAppliedVolts);
    table.put("EndEffectorCurrentAmps", EndEffectorCurrentAmps);
  
  }

  @Override
  public void fromLog(LogTable table) {
    EndEffectorConnected = table.get("EndEffectorConnected", EndEffectorConnected);
    EndEffectorAppliedVolts = table.get("EndEffectorAppliedVolts", EndEffectorAppliedVolts);
    EndEffectorCurrentAmps = table.get("EndEffectorCurrentAmps", EndEffectorCurrentAmps);

  }

  public EndEffectorIOInputsAutoLogged clone() {
    EndEffectorIOInputsAutoLogged copy = new EndEffectorIOInputsAutoLogged();
    copy.EndEffectorConnected = this.EndEffectorConnected;
    copy.EndEffectorAppliedVolts = this.EndEffectorAppliedVolts;
    copy.EndEffectorCurrentAmps = this.EndEffectorCurrentAmps;
    return copy;
  }
}
