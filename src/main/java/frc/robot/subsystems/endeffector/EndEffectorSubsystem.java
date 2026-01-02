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
  private final Alert EndEffectorDisconnectedAlert;

  public EndEffectorSubsystem(EndEffectorIO io) {
    this.io = io;
    EndEffectorDisconnectedAlert =
        new Alert("Disconnected End Effector motor" + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
    EndEffectorDisconnectedAlert.set(!inputs.EndEffectorConnected);
  }

  public Command intake(double output) {
    return Commands.run(() -> io.setEndEffectorOpenLoop(output));
  }

  public void stop() {
    io.setEndEffectorOpenLoop(0);
  }
}
