// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Comando para controlar el elevador manualmente a una velocidad constante,
 * respetando los limit switches de arriba y abajo.
 */
public class ElevatorCmd extends Command {
  private final Elevator elevator;
  private final double speed;
  private final DigitalInput downLimitSwitch, upLimitSwitch;

  /**
   * Crea un nuevo comando para el elevador con limit switches.
   * @param elevator Subsistema del elevador
   * @param speed Velocidad a aplicar (-1 a 1)
   * @param downLimitSwitch Limit switch inferior
   * @param upLimitSwitch Limit switch superior
   */
  public ElevatorCmd(Elevator elevator, double speed, DigitalInput downLimitSwitch, DigitalInput upLimitSwitch) {
    this.elevator = elevator;
    this.speed = speed;
    this.downLimitSwitch = downLimitSwitch;
    this.upLimitSwitch = upLimitSwitch;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.set_speed(0);
  }

  @Override
  public void execute() {
    // Si bajando y el switch de abajo está presionado, no mover
    if (speed < 0 && downLimitSwitch.get()) {
      elevator.set_speed(0);
    }
    // Si subiendo y el switch de arriba está presionado, no mover
    else if (speed > 0 && upLimitSwitch.get()) {
      elevator.set_speed(0);
    }
    else {
      elevator.set_speed(speed);
    }
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