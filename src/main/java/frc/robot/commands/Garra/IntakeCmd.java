package frc.robot.commands.Garra;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * COMANDO: IntakeCmd
 * Prende el intake a la velocidad configurada mientras se mantiene presionado el botón.
 */
public class IntakeCmd extends Command {
  private final Intake intake;
  private final double speed;

  public IntakeCmd(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}