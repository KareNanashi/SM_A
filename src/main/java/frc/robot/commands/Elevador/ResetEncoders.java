// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * Comando para restablecer los encoders del elevador.
 */
public class ResetEncoders extends Command {
  private final Elevator elevator;
  
  /**
   * Crea un nuevo comando para restablecer encoders.
   * @param elevator Subsistema del elevador
   */
  public ResetEncoders(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.reset_encoders();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true; // Comando instantáneo
  }
}