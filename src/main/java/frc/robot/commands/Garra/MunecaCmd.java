// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Garra;

import frc.robot.subsystems.Muneca;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * COMANDO: MunecaCmd
 * Controla la muñeca de la garra con el valor de un eje del control (proporcional y con zona muerta antidrift).
 */
public class MunecaCmd extends Command {
  private final Muneca muneca;
  private final Supplier<Double> ejeValue;

  public MunecaCmd(Muneca muneca, Supplier<Double> ejeValue) {
    this.muneca = muneca;
    this.ejeValue = ejeValue;
    addRequirements(muneca);
  }

  @Override
  public void execute() {
    double value = ejeValue.get();
    // Zona muerta antidrift: ignora movimientos menores a ±0.07
    double output = Math.abs(value) > 0.07 ? value : 0.0;
    muneca.setSpeed(output);
  }

  @Override
  public void end(boolean interrupted) {
    muneca.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}