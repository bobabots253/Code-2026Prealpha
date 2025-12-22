package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class EndEffectorSubsystem {
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private final int index;
  private final Alert EndEffectorDisconnectedAlert;

  public EndEffectorSubsystem(EndEffectorIO io, int index) {
    this.io = io;
    this.index = index;
    EndEffectorDisconnectedAlert =
        new Alert(
            "Disconnected End Effector motor" + Integer.toString(index) + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    EndEffectorDisconnectedAlert.set(!inputs.EndEffectorConnected);
  }

  public Command intake(double output) {
    return Commands.run(() -> io.setEndEffectorOpenLoop(output));
  }

  public void stop() {
    io.setEndEffectorOpenLoop(0);
  }
}
