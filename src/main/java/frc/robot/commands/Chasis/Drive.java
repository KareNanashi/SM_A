// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/**
 * COMANDO: Drive
 * Permite que el robot avance en línea recta una distancia específica.
 * Parámetros:
 * - targetDistance: Distancia objetivo en metros.
 * - speed: Velocidad constante a aplicar (-1 a 1).
 * El comando termina cuando la distancia recorrida >= objetivo.
 */
public class Drive extends Command {
  private final Chasis chasis;
  private final double targetDistance;
  private final double speed;
  
  /**
   * Constructor del comando de avance recto.
   * @param chasis Subsistema de chasis
   * @param targetDistance Distancia objetivo en metros
   * @param speed Velocidad a utilizar (-1 a 1)
   */
  public Drive(Chasis chasis, double targetDistance, double speed) {
    this.chasis = chasis;
    this.targetDistance = targetDistance;
    this.speed = speed;
    addRequirements(chasis);
  }

  /** Inicializa: resetea encoders al iniciar. */
  @Override
  public void initialize() {
    chasis.resetEncoder();
  }

  /** Comanda avance recto a velocidad constante. */
  @Override
  public void execute() {
    chasis.set_motors(speed, speed);
  }

  /** Detiene motores al terminar o ser interrumpido. */
  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
  }

  /** Termina cuando el robot ha recorrido la distancia objetivo. */
  @Override
  public boolean isFinished() {
    return chasis.getAverageEncoderDistance() >= targetDistance;
  }
}