// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/**
 * Comando para conducir en línea recta una distancia específica.
 */
public class Drive extends Command {
  private final Chasis chasis;
  private final double targetDistance;
  private final double speed;
  
  /**
   * Crea un nuevo comando para conducir en línea recta.
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