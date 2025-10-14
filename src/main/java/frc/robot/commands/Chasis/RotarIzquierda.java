// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/**
 * COMANDO: RotarIzquierda
 * Permite que el robot gire sobre su eje hacia la izquierda una cantidad de grados especificada.
 * Parámetros:
 * - rightSpeed: Velocidad del lado derecho.
 * - leftSpeed: Velocidad del lado izquierdo.
 * - degrees: Grados a rotar.
 * El comando termina cuando el encoder derecho alcanza la distancia de rotación calculada.
 */
public class RotarIzquierda extends Command {
  private final Chasis chasis;
  private final double rightSpeed;
  private final double leftSpeed;
  private final double degrees;
  
  /**
   * Constructor del comando de giro a la izquierda.
   * @param chasis Subsistema de chasis
   * @param rightSpeed Velocidad del lado derecho
   * @param leftSpeed Velocidad del lado izquierdo
   * @param degrees Grados a rotar
   */
  public RotarIzquierda(Chasis chasis, double rightSpeed, double leftSpeed, double degrees) {
    this.chasis = chasis;
    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;
    this.degrees = degrees;
    addRequirements(chasis);
  }

  /** Inicializa: resetea encoder al iniciar. */
  @Override
  public void initialize() {
    chasis.resetEncoder();
  }

  /** Aplica velocidades para girar sobre eje. */
  @Override
  public void execute() {
    chasis.set_motors(leftSpeed, rightSpeed);
  }

  /** Detiene motores al terminar o ser interrumpido. */
  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
  }

  /** Termina cuando el encoder derecho alcanza la distancia de rotación calculada. */
  @Override
  public boolean isFinished() {
    double currentDistance = chasis.get_right_encoder_distance();
    double targetDistance = chasis.rotacion(degrees);
    return currentDistance >= targetDistance;
  }
}