// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/**
 * COMANDO: AutonomoAvanzar
 * Comando pensado para uso en modo autónomo que únicamente avanza hacia adelante.
 * Parámetros:
 * - targetDistance: distancia objetivo en metros.
 * - speed: velocidad de avance (valor positivo entre 0 y 1).
 *
 * Uso típico en auton: new AutonomoAvanzar(chasis, 2.0, 0.5) para avanzar 2 metros al 50% de potencia.
 */
public class AutonomoAvanzar extends Command {
  private final Chasis chasis;
  private final double targetDistance;
  private final double speed;

  public AutonomoAvanzar(Chasis chasis, double targetDistance, double speed) {
    this.chasis = chasis;
    this.targetDistance = targetDistance;
    this.speed = Math.abs(speed); // Forzamos positivo: avanzar hacia adelante
    addRequirements(chasis);
  }

  @Override
  public void initialize() {
    chasis.resetEncoder();
  }

  @Override
  public void execute() {
    chasis.set_motors(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
  }

  @Override
  public boolean isFinished() {
    return chasis.getAverageEncoderDistance() >= targetDistance;
  }
}