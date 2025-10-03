// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Vision;

/**
 * Command to continuously search for and follow AprilTags.
 * Ahora si el tag se pierde, el robot sigue la última posición suavizada.
 */
public class ContinuousAprilTagFollowCommand extends Command {
  private final Vision vision;
  private final Chasis chasis;
  private final PIDController rotationPID;
  private final PIDController distancePID;

  // PID constants - tunables via SmartDashboard
  private static final double DEFAULT_ROTATION_P = 0.015;
  private static final double DEFAULT_ROTATION_I = 0.001;
  private static final double DEFAULT_ROTATION_D = 0.0005;

  private static final double DEFAULT_DISTANCE_P = 0.4;
  private static final double DEFAULT_DISTANCE_I = 0.01;
  private static final double DEFAULT_DISTANCE_D = 0.02;

  // Target distance to maintain from tag (in meters)
  private static final double DEFAULT_TARGET_DISTANCE = 1.0;

  // Suavizado
  private static final double SMOOTHING_ALPHA = 0.3; // 0 = sin suavizado, 1 = sin efecto

  // Control variables
  private double targetDistance;
  private boolean isSearching = false;

  // Última posición/ángulo válidos (suavizados)
  private double lastKnownAngle = 0.0;
  private double lastKnownDistance = DEFAULT_TARGET_DISTANCE;
  private int lastKnownTagID = -1;

  // Ciclos sin ver el tag
  private int noTargetCounter = 0;
  private static final int MAX_NO_TARGET_COUNT = 75; // ~1.5 segundos (20ms por ciclo)

  public ContinuousAprilTagFollowCommand(Vision vision, Chasis chasis) {
    this.vision = vision;
    this.chasis = chasis;

    this.rotationPID = new PIDController(DEFAULT_ROTATION_P, DEFAULT_ROTATION_I, DEFAULT_ROTATION_D);
    this.distancePID = new PIDController(DEFAULT_DISTANCE_P, DEFAULT_DISTANCE_I, DEFAULT_DISTANCE_D);

    this.targetDistance = DEFAULT_TARGET_DISTANCE;

    addRequirements(vision, chasis);
  }

  @Override
  public void initialize() {
    chasis.resetEncoder();
    SmartDashboard.putNumber("Vision/Rotation P", rotationPID.getP());
    SmartDashboard.putNumber("Vision/Rotation I", rotationPID.getI());
    SmartDashboard.putNumber("Vision/Rotation D", rotationPID.getD());
    SmartDashboard.putNumber("Vision/Distance P", distancePID.getP());
    SmartDashboard.putNumber("Vision/Distance I", distancePID.getI());
    SmartDashboard.putNumber("Vision/Distance D", distancePID.getD());
    SmartDashboard.putNumber("Vision/Target Distance", targetDistance);

    isSearching = false;
    noTargetCounter = 0;

    // Inicializar últimos valores suavizados
    lastKnownAngle = 0.0;
    lastKnownDistance = targetDistance;
    lastKnownTagID = -1;

    System.out.println("Continuous AprilTag following initialized");
    SmartDashboard.putString("Vision/Status", "Searching for AprilTags");
  }

  @Override
  public void execute() {
    updatePIDFromDashboard();

    // Suavizado: si hay tag, mezclar valor nuevo con el anterior
    if (vision.hasTarget()) {
      noTargetCounter = 0;

      double detectedAngle = vision.getZAngle();
      double detectedDistance = vision.getXAxis();

      // Suavizado exponencial para evitar movimientos bruscos
      lastKnownAngle = SMOOTHING_ALPHA * detectedAngle + (1 - SMOOTHING_ALPHA) * lastKnownAngle;
      lastKnownDistance = SMOOTHING_ALPHA * detectedDistance + (1 - SMOOTHING_ALPHA) * lastKnownDistance;
      lastKnownTagID = vision.getIDApriltag();

      SmartDashboard.putNumber("Vision/Current Tag ID", lastKnownTagID);
    } else {
      noTargetCounter++;
    }

    // Si se perdió el tag por mucho tiempo, detente
    if (noTargetCounter > MAX_NO_TARGET_COUNT) {
      chasis.set_motors(0, 0);
      SmartDashboard.putString("Vision/Status", "Tag lost, stopping");
      return;
    }

    // Usa la última posición suavizada para seguir (aunque no haya tag visible)
    double currentAngle = lastKnownAngle;
    double currentDistance = lastKnownDistance;

    double rotationOutput = -rotationPID.calculate(currentAngle, 0);
    double distanceOutput = distancePID.calculate(currentDistance, targetDistance);

    rotationOutput = clamp(rotationOutput, -0.5, 0.5);
    distanceOutput = clamp(distanceOutput, -0.5, 0.5);

    double leftPower = distanceOutput + rotationOutput;
    double rightPower = distanceOutput - rotationOutput;

    chasis.set_motors(leftPower, rightPower);

    // Debug info
    SmartDashboard.putNumber("Vision/Rotation Output", rotationOutput);
    SmartDashboard.putNumber("Vision/Distance Output", distanceOutput);
    String status = vision.hasTarget()
        ? "Following Tag ID: " + lastKnownTagID
        : "Lost, going to last known (smoothed) position";
    SmartDashboard.putString("Vision/Status", status);
  }

  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
    rotationPID.reset();
    distancePID.reset();

    String endReason = interrupted ? "interrupted" : "completed";
    System.out.println("AprilTag following ended: " + endReason);
    SmartDashboard.putString("Vision/Status", "Idle");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void updatePIDFromDashboard() {
    try {
      double rP = SmartDashboard.getNumber("Vision/Rotation P", rotationPID.getP());
      double rI = SmartDashboard.getNumber("Vision/Rotation I", rotationPID.getI());
      double rD = SmartDashboard.getNumber("Vision/Rotation D", rotationPID.getD());
      double dP = SmartDashboard.getNumber("Vision/Distance P", distancePID.getP());
      double dI = SmartDashboard.getNumber("Vision/Distance I", distancePID.getI());
      double dD = SmartDashboard.getNumber("Vision/Distance D", distancePID.getD());
      targetDistance = SmartDashboard.getNumber("Vision/Target Distance", targetDistance);

      if (rP != rotationPID.getP() || rI != rotationPID.getI() || rD != rotationPID.getD()) {
        rotationPID.setPID(rP, rI, rD);
      }
      if (dP != distancePID.getP() || dI != distancePID.getI() || dD != distancePID.getD()) {
        distancePID.setPID(dP, dI, dD);
      }
    } catch (Exception e) {
      // Ignore if values aren't on dashboard yet
    }
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}