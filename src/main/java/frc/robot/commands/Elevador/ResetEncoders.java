// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * COMANDO: ResetEncoders
 * Comando instantáneo para resetear el encoder del elevador.
 * Se utiliza como botón en el Dashboard o como acción puntual.
 */
public class ResetEncoders extends Command {
  private final Elevator elevator;
  
  /** Constructor: requiere el subsistema Elevator */
  public ResetEncoders(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  /** Resetea encoder al iniciar el comando. */
  @Override
  public void initialize() {
    elevator.reset_encoders();
  }

  /** No hace nada en execute. */
  @Override
  public void execute() {}

  /** No hace nada al terminar. */
  @Override
  public void end(boolean interrupted) {}

  /** Comando termina inmediatamente. */
  @Override
  public boolean isFinished() {
    return true;
  }
}