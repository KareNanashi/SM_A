// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Comando para controlar el intake manualmente.
 */
public class IntakeCmd extends Command {
  private final Intake intake;
  private final Supplier<Double> speedSupplier;
  private static final double DEADBAND = 0.35;
  
  /**
   * Crea un nuevo comando para el intake.
   * @param intake Subsistema del intake
   * @param speedSupplier Proveedor de velocidad (-1 a 1)
   */
  public IntakeCmd(Intake intake, Supplier<Double> speedSupplier) {
    this.intake = intake;
    this.speedSupplier = speedSupplier;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.set_speed(0);
  }

  @Override
  public void execute() {
    double speed = speedSupplier.get();
    
    // Aplicar zona muerta para evitar drift
    if (Math.abs(speed) < DEADBAND) {
      speed = 0;
    }
    
    intake.set_speed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.set_speed(0);
  }

  @Override
  public boolean isFinished() {
    return false; // Este comando se ejecuta hasta que sea interrumpido
  }
}