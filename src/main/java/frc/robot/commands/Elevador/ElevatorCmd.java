// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * Comando para controlar el elevador manualmente a una velocidad constante.
 */
public class ElevatorCmd extends Command {
  private final Elevator elevator;
  private final double speed;
  
  /**
   * Crea un nuevo comando para el elevador.
   * @param elevator Subsistema del elevador
   * @param speed Velocidad a aplicar (-1 a 1)
   */
  public ElevatorCmd(Elevator elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.set_speed(0);
  }

  @Override
  public void execute() {
    elevator.set_speed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.set_speed(0);
  }

  @Override
  public boolean isFinished() {
    return false; // Este comando se ejecuta hasta que sea interrumpido
  }
}
