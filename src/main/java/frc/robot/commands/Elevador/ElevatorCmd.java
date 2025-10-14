// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * COMANDO: ElevatorCmd
 * Permite controlar el elevador manualmente a una velocidad constante
 * respetando los limit switches de arriba y abajo.
 * Parámetros:
 * - speed: Velocidad deseada (-1 a 1).
 * - downLimitSwitch/upLimitSwitch: Limit switches físicos.
 */
public class ElevatorCmd extends Command {
  private final Elevator elevator;
  private final double speed;
  private final DigitalInput downLimitSwitch, upLimitSwitch;

  /**
   * Constructor: configura elevador y switches.
   * @param elevator Subsistema del elevador
   * @param speed Velocidad de movimiento
   * @param downLimitSwitch Switch inferior
   * @param upLimitSwitch Switch superior
   */
  public ElevatorCmd(Elevator elevator, double speed, DigitalInput downLimitSwitch, DigitalInput upLimitSwitch) {
    this.elevator = elevator;
    this.speed = speed;
    this.downLimitSwitch = downLimitSwitch;
    this.upLimitSwitch = upLimitSwitch;
    addRequirements(elevator);
  }

  /** Inicializa elevador detenido. */
  @Override
  public void initialize() {
    elevator.set_speed(0);
  }

  /**
   * Ejecuta movimiento, pero lo detiene si el limit switch correspondiente está activado.
   * Cambiado: ahora el Falcon se detiene SOLO cuando el switch está ACTIVADO (imán cerca).
   * Si el switch está desactivado (imán lejos), el Falcon puede moverse.
   */
  @Override
  public void execute() {
    if (speed < 0 && !downLimitSwitch.get()) {
      elevator.set_speed(0);
    }
    else if (speed > 0 && !upLimitSwitch.get()) {
      elevator.set_speed(0);
    }
    else {
      elevator.set_speed(speed);
    }
  }

  /** Detiene el elevador al terminar o ser interrumpido. */
  @Override
  public void end(boolean interrupted) {
    elevator.set_speed(0);
  }

  /** El comando nunca termina por sí mismo. */
  @Override
  public boolean isFinished() {
    return false;
  }
}