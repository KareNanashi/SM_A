// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/**
 * Comando para conducción en modo arcade (velocidad + giro).
 */
public class ArcadeDriveCmd extends Command {
  private final Chasis chasis;
  private final Supplier<Double> speedFunction, turnFunction;
  
  /**
   * Crea un nuevo comando de conducción arcade.
   * @param chasis Subsistema de chasis
   * @param speedFunction Proveedor de velocidad lineal (-1 a 1)
   * @param turnFunction Proveedor de velocidad de giro (-1 a 1)
   */
  public ArcadeDriveCmd(Chasis chasis, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    this.chasis = chasis;
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    addRequirements(chasis);
  }

  @Override
  public void initialize() {
    chasis.set_motors(0, 0);
  }

  @Override
  public void execute() {
    double realTimeSpeed = speedFunction.get();
    double realTimeTurn = turnFunction.get();

    // Calcular potencias para cada lado
    double left = realTimeSpeed + realTimeTurn;
    double right = realTimeSpeed - realTimeTurn;
    
    // Eliminar drift del motor (zona muerta)
    if (Math.abs(left) < 0.06) {
      left = 0;
    }
    if (Math.abs(right) < 0.06) {
      right = 0;
    }
    
    chasis.set_motors(left, right);
  }

  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(1.5, 1.5);
  }

  @Override
  public boolean isFinished() {
    return false; // Este comando se ejecuta hasta que sea interrumpido
  }
}