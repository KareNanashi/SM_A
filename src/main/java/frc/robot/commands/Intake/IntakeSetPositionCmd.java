// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Comando para mover el intake a una posición específica usando PID.
 */
public class IntakeSetPositionCmd extends Command {
  private final Intake intake;
  private final double targetPosition;
  
  /**
   * Crea un nuevo comando para posicionar el intake.
   * @param intake Subsistema del intake
   * @param targetPosition Posición objetivo
   */
  public IntakeSetPositionCmd(Intake intake, double targetPosition) {
    this.intake = intake;
    this.targetPosition = targetPosition;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.goPosition(targetPosition);
    SmartDashboard.putNumber("Intake/Target Position", targetPosition);
  }

  @Override
  public void execute() {
    intake.goPosition(targetPosition);
    
    // Mostrar información de depuración
    double currentPosition = intake.getCurrentPosition();
    SmartDashboard.putNumber("Intake/Current Position", currentPosition);
    SmartDashboard.putNumber("Intake/Position Error", targetPosition - currentPosition);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double currentPosition = intake.getCurrentPosition();
    return (currentPosition + 1) >= targetPosition;
  }
}