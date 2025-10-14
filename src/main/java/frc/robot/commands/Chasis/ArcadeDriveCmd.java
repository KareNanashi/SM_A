// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/**
 * COMANDO: ArcadeDriveCmd
 * Permite controlar el chasis en modo "arcade" con velocidad y giro combinados.
 * Se usa como default command para manejo en teleoperado.
 * Parámetros:
 * - speedFunction: Proveedor de velocidad lineal (-1 a 1).
 * - turnFunction: Proveedor de velocidad angular (-1 a 1).
 */
public class ArcadeDriveCmd extends Command {
  private final Chasis chasis;
  private final Supplier<Double> speedFunction, turnFunction;
  
  /**
   * Constructor del comando de conducción arcade.
   * @param chasis Subsistema de chasis
   * @param speedFunction Proveedor de velocidad lineal (-1 a 1)
   * @param turnFunction Proveedor de giro (-1 a 1)
   */
  public ArcadeDriveCmd(Chasis chasis, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    this.chasis = chasis;
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    addRequirements(chasis);
  }

  /** Inicializa y detiene motores al empezar el comando. */
  @Override
  public void initialize() {
    chasis.set_motors(0, 0);
  }

  /** 
   * Se ejecuta cada ciclo: calcula las velocidades en tiempo real
   * y comanda los motores. Aplica zona muerta para evitar drift.
   */
  @Override
  public void execute() {
    double realTimeSpeed = speedFunction.get();
    double realTimeTurn = turnFunction.get();

    // Calcula potencias para cada lado (arcade)
    double left = realTimeSpeed + realTimeTurn;
    double right = realTimeSpeed - realTimeTurn;
    
    // Zona muerta para minimizar drift por errores menores
    if (Math.abs(left) < 0.06) {
      left = 0;
    }
    if (Math.abs(right) < 0.06) {
      right = 0;
    }
    
    chasis.set_motors(left, right);
  }

  /** Detiene los motores al finalizar el comando (o ser interrumpido). */
  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
  }

  /** El comando nunca termina por sí mismo (default command). */
  @Override
  public boolean isFinished() {
    return false;
  }
}