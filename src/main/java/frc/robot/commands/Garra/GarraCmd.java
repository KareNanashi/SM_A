// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Garra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Garra;

/**
 * Comando para controlar la garra a velocidades específicas.
 */
public class GarraCmd extends Command {
  private final Garra garra;
  private final double falcon_speed; // Velocidad para el motor Falcon (tubos)
  private final double redline_speed; // Velocidad para el motor RedLine (pelotas)

  /**
   * Crea un nuevo comando para la garra.
   * @param garra Subsistema de la garra
   * @param falcon_speed Velocidad para el motor Falcon (-1 a 1)
   * @param redline_speed Velocidad para el motor RedLine (-1 a 1)
   */
  public GarraCmd(Garra garra, double falcon_speed, double redline_speed) {
    this.garra = garra;
    this.falcon_speed = falcon_speed;
    this.redline_speed = redline_speed;
    addRequirements(garra);
  }

  @Override
  public void initialize() {
    garra.set_motor_garra(0, 0);
  }

  @Override
  public void execute() {
    garra.set_motor_garra(falcon_speed, redline_speed);
  }

  @Override
  public void end(boolean interrupted) {
    garra.set_motor_garra(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false; // Este comando se ejecuta hasta que sea interrumpido
  }
}