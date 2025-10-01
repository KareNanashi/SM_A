// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/**
 * Comando para rotar el robot hacia la derecha un ángulo específico.
 */
public class RotarDerecha extends Command {
  private final Chasis chasis;
  private final double rightSpeed;
  private final double leftSpeed;
  private final double degrees;
  
  /**
   * Crea un nuevo comando para rotar a la derecha.
   * @param chasis Subsistema de chasis
   * @param rightSpeed Velocidad del lado derecho
   * @param leftSpeed Velocidad del lado izquierdo
   * @param degrees Grados a rotar
   */
  public RotarDerecha(Chasis chasis, double rightSpeed, double leftSpeed, double degrees) {
    this.chasis = chasis;
    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;
    this.degrees = degrees;
    addRequirements(chasis);
  }

  @Override
  public void initialize() {
    chasis.resetEncoder();
  }

  @Override
  public void execute() {
    chasis.set_motors(leftSpeed, rightSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
  }

  @Override
  public boolean isFinished() {
    double currentDistance = chasis.get_left_encoder_distance();
    double targetDistance = chasis.rotacion(degrees);
    return currentDistance >= targetDistance;
  }
}