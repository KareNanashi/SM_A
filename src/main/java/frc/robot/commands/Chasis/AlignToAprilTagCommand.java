// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Vision;

/**
 * COMANDO: AlignToAprilTagCommand
 * Permite al robot alinearse al ángulo Z de un AprilTag usando información de visión.
 * Si el tag se pierde, la rotación decae gradualmente a cero.
 * Utiliza PID para calcular la corrección de giro.
 */
public class AlignToAprilTagCommand extends Command {
  private final Vision vision;
  private final Chasis chasis;
  private final PIDController rotationPID;

  // Constantes de PID para la alineación angular
  private static final double ROTATION_P = 0.01;
  private static final double ROTATION_I = 0.0005;
  private static final double ROTATION_D = 0.0001;
  private static final double MAX_ROTATION_OUTPUT = 0.2;

  // Suavizado progresivo cuando se pierde el tag
  private double fadeMultiplier = 1.0;
  private static final double FADE_STEP = 0.06; // Decae de 1.0 a 0.0 en ~16 ciclos (0.3 seg)
  private static final int FADE_CYCLE_LIMIT = 20; // Luego de 20 ciclos sin tag, se detiene

  private int noTagCounter = 0;
  private double lastKnownAngle = 0.0;

  /**
   * Constructor del comando de alineación.
   * @param vision Subsistema de visión con PhotonVision
   * @param chasis Subsistema de chasis
   */
  public AlignToAprilTagCommand(Vision vision, Chasis chasis) {
    this.vision = vision;
    this.chasis = chasis;
    this.rotationPID = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
    addRequirements(vision, chasis);
  }

  /** Inicializa contadores y PID al comenzar alineación. */
  @Override
  public void initialize() {
    fadeMultiplier = 1.0;
    noTagCounter = 0;
    lastKnownAngle = 0.0;
    rotationPID.reset();
    SmartDashboard.putString("Vision/Status", "Aligning to AprilTag");
  }

  /**
   * Se ejecuta cada ciclo:
   * - Si hay tag, actualiza ángulo y PID.
   * - Si se pierde el tag, decae la rotación.
   * - Detiene giro si pasan demasiados ciclos sin tag.
   */
  @Override
  public void execute() {
    if (vision.hasTarget()) {
      lastKnownAngle = vision.getZAngle();
      noTagCounter = 0;
      fadeMultiplier = 1.0;
    } else {
      noTagCounter++;
      fadeMultiplier = Math.max(0.0, fadeMultiplier - FADE_STEP);
    }

    if (noTagCounter > FADE_CYCLE_LIMIT) {
      chasis.set_motors(0, 0);
      SmartDashboard.putString("Vision/Status", "Tag lost, stopped aligning");
      return;
    }

    double rotationOutput = -rotationPID.calculate(lastKnownAngle, 0) * fadeMultiplier;
    rotationOutput = clamp(rotationOutput, -MAX_ROTATION_OUTPUT, MAX_ROTATION_OUTPUT);

    chasis.set_motors(rotationOutput, -rotationOutput);

    SmartDashboard.putNumber("Vision/Align Rotation Output", rotationOutput);
    SmartDashboard.putString("Vision/Status",
      vision.hasTarget() ? "Aligning to Tag" : "Lost Tag, fading alignment");
  }

  /** Detiene motores y resetea PID al finalizar o ser interrumpido. */
  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
    rotationPID.reset();
    SmartDashboard.putString("Vision/Status", "Alignment stopped");
  }

  /** El comando nunca termina por sí mismo. */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Función auxiliar para limitar valores. */
  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}