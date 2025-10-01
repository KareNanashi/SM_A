// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeSetPositionCmd;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/**
 * Secuencia que coordina el movimiento del elevador y el intake.
 * Ejecuta primero el movimiento del elevador y luego el del intake.
 */
public class ElevatorIntakeSequence extends SequentialCommandGroup {
  /**
   * Crea una nueva secuencia coordinada de elevador e intake.
   * @param elevator Subsistema del elevador
   * @param intake Subsistema del intake
   * @param elevatorTarget Posición objetivo del elevador
   * @param intakeTarget Posición objetivo del intake
   */
  public ElevatorIntakeSequence(Elevator elevator, Intake intake, double elevatorTarget, double intakeTarget) {
    addCommands(
        new MoveElevatorToPosition(elevator, elevatorTarget),
        new IntakeSetPositionCmd(intake, intakeTarget)
    );
  }
}