package frc.robot.commands.Chasis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Vision;

/**
 * Command to align the robot to the AprilTag's angle (Z) only.
 * If tag is lost, rotation is reduced smoothly to zero.
 */
public class AlignToAprilTagCommand extends Command {
  private final Vision vision;
  private final Chasis chasis;
  private final PIDController rotationPID;

  // PID constants
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

  public AlignToAprilTagCommand(Vision vision, Chasis chasis) {
    this.vision = vision;
    this.chasis = chasis;
    this.rotationPID = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
    addRequirements(vision, chasis);
  }

  @Override
  public void initialize() {
    fadeMultiplier = 1.0;
    noTagCounter = 0;
    lastKnownAngle = 0.0;
    rotationPID.reset();
    SmartDashboard.putString("Vision/Status", "Aligning to AprilTag");
  }

  @Override
  public void execute() {
    // Si hay tag, actualiza el ángulo y multiplica la salida normal
    if (vision.hasTarget()) {
      lastKnownAngle = vision.getZAngle();
      noTagCounter = 0;
      fadeMultiplier = 1.0;
    } else {
      // Si no hay tag, empieza a decaer el giro
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

    // Mostrar datos en dashboard
    SmartDashboard.putNumber("Vision/Align Rotation Output", rotationOutput);
    SmartDashboard.putString("Vision/Status",
      vision.hasTarget() ? "Aligning to Tag" : "Lost Tag, fading alignment");
  }

  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
    rotationPID.reset();
    SmartDashboard.putString("Vision/Status", "Alignment stopped");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}