// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * Comando para mover el elevador a una posición específica usando PID.
 */
public class MoveElevatorToPosition extends Command {
    private final Elevator elevator;
    private final double targetPosition;
    private static final double POSITION_TOLERANCE = 2.0; // Tolerancia en cm

    /**
     * Crea un nuevo comando para mover el elevador a una posición específica.
     * @param elevator Subsistema del elevador
     * @param targetPosition Posición objetivo
     */
    public MoveElevatorToPosition(Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.ElevatorGoPosition(targetPosition);
        SmartDashboard.putNumber("Elevator/Target Position", targetPosition);
    }

    @Override
    public void execute() {
        elevator.ElevatorGoPosition(targetPosition);
        
        // Mostrar información de depuración
        double currentPosition = elevator.getCurrentPosition();
        double error = targetPosition - currentPosition;
        
        SmartDashboard.putNumber("Elevator/Current Position", currentPosition);
        SmartDashboard.putNumber("Elevator/Position Error", error);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = elevator.getCurrentPosition();
        boolean inPosition = Math.abs(currentPosition - targetPosition) < POSITION_TOLERANCE;
        
        if (inPosition) {
            SmartDashboard.putString("Elevator/Status", "En posición");
        } else {
            SmartDashboard.putString("Elevator/Status", "Moviéndose");
        }
        
        return inPosition;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putString("Elevator/Status", "Interrumpido");
        }
    }
}