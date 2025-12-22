package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffectorSubsystem extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private final int index;

  private final Alert endEffectorDisconnectedAlert;

  public EndEffectorSubsystem(EndEffectorIO io, int index) {
    this.io = io;
    this.index = index;
    endEffectorDisconnectedAlert =
        new Alert(
            "Disconnected End Effector Motor " + Integer.toString(index) + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("End Effector" + Integer.toString(index), inputs);
    endEffectorDisconnectedAlert.set(!inputs.endEffectorConnected);
  }

  public Command setOpenLoop(int value) {
    return Commands.run(() -> io.setEndEffectorOpenLoop(value));
  }
}
