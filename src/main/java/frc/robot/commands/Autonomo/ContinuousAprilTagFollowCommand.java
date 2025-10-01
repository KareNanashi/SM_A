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
 * Unlike the alignment command, this will never finish and will
 * continuously track tags as they appear.
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
  
  // Search rotation speed when no target is found
  private static final double SEARCH_ROTATION_SPEED = 0.2;
  
  // Control variables
  private double targetDistance;
  private boolean isSearching = false;
  private int noTargetCounter = 0;
  private static final int MAX_NO_TARGET_COUNT = 25; // ~0.5 seconds without target before searching
  
  /**
   * Creates a command to continuously follow AprilTags.
   * 
   * @param vision The vision subsystem
   * @param chasis The drive subsystem
   */
  public ContinuousAprilTagFollowCommand(Vision vision, Chasis chasis) {
    this.vision = vision;
    this.chasis = chasis;
    
    // Initialize PID controllers
    this.rotationPID = new PIDController(DEFAULT_ROTATION_P, DEFAULT_ROTATION_I, DEFAULT_ROTATION_D);
    this.distancePID = new PIDController(DEFAULT_DISTANCE_P, DEFAULT_DISTANCE_I, DEFAULT_DISTANCE_D);
    
    // Set default target distance
    this.targetDistance = DEFAULT_TARGET_DISTANCE;
    
    addRequirements(vision, chasis);
  }
  
  @Override
  public void initialize() {
    // Reset encoders
    chasis.resetEncoder();
    
    // Publish PID values to SmartDashboard for tuning
    SmartDashboard.putNumber("Vision/Rotation P", rotationPID.getP());
    SmartDashboard.putNumber("Vision/Rotation I", rotationPID.getI());
    SmartDashboard.putNumber("Vision/Rotation D", rotationPID.getD());
    
    SmartDashboard.putNumber("Vision/Distance P", distancePID.getP());
    SmartDashboard.putNumber("Vision/Distance I", distancePID.getI());
    SmartDashboard.putNumber("Vision/Distance D", distancePID.getD());
    
    SmartDashboard.putNumber("Vision/Target Distance", targetDistance);
    
    // Reset state
    isSearching = false;
    noTargetCounter = 0;
    
    System.out.println("Continuous AprilTag following initialized");
    SmartDashboard.putString("Vision/Status", "Searching for AprilTags");
  }

  @Override
  public void execute() {
    // Update PID values from dashboard (for tuning)
    updatePIDFromDashboard();
    
    // Display the ID of the currently tracked tag (if any)
    if (vision.hasTarget()) {
      SmartDashboard.putNumber("Vision/Current Tag ID", vision.getIDApriltag());
    }
    
    if (!vision.hasTarget()) {
      handleNoTarget();
      return;
    }
    
    // We have a target, reset search state
    isSearching = false;
    noTargetCounter = 0;
    
    // Get current measurements
    double currentAngle = vision.getZAngle();
    double currentDistance = vision.getXAxis();
    
    // Calculate PID outputs
    double rotationOutput = -rotationPID.calculate(currentAngle, 0);
    double distanceOutput = distancePID.calculate(currentDistance, targetDistance);
    
    // Clamp outputs
    rotationOutput = clamp(rotationOutput, -0.5, 0.5);
    distanceOutput = clamp(distanceOutput, -0.5, 0.5);
    
    // Drive the robot with PID outputs
    double leftPower = distanceOutput + rotationOutput;
    double rightPower = distanceOutput - rotationOutput;
    
    chasis.set_motors(leftPower, rightPower);
    
    // Display debug info
    SmartDashboard.putNumber("Vision/Rotation Output", rotationOutput);
    SmartDashboard.putNumber("Vision/Distance Output", distanceOutput);
    SmartDashboard.putString("Vision/Status", "Following Tag ID: " + vision.getIDApriltag());
  }
  
  /**
   * Handles the case when no target is detected.
   */
  private void handleNoTarget() {
    noTargetCounter++;
    
    if (noTargetCounter > MAX_NO_TARGET_COUNT) {
      // Start searching after a delay without target
      isSearching = true;
      
      // Rotate to search for targets
      chasis.set_motors(SEARCH_ROTATION_SPEED, -SEARCH_ROTATION_SPEED);
      
      SmartDashboard.putString("Vision/Status", "Searching");
    } else {
      // Briefly maintain last control outputs
      SmartDashboard.putString("Vision/Status", "Target Lost - Continuing");
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop motors
    chasis.set_motors(0, 0);
    
    // Reset controllers
    rotationPID.reset();
    distancePID.reset();
    
    String endReason = interrupted ? "interrupted" : "completed";
    System.out.println("AprilTag following ended: " + endReason);
    SmartDashboard.putString("Vision/Status", "Idle");
  }

  @Override
  public boolean isFinished() {
    // This command never finishes on its own - it continuously follows tags
    return false;
  }
  
  /**
   * Updates PID constants from SmartDashboard values.
   * This allows tuning without recompiling.
   */
  private void updatePIDFromDashboard() {
    // Only update if the values exist on dashboard
    try {
      double rP = SmartDashboard.getNumber("Vision/Rotation P", rotationPID.getP());
      double rI = SmartDashboard.getNumber("Vision/Rotation I", rotationPID.getI());
      double rD = SmartDashboard.getNumber("Vision/Rotation D", rotationPID.getD());
      
      double dP = SmartDashboard.getNumber("Vision/Distance P", distancePID.getP());
      double dI = SmartDashboard.getNumber("Vision/Distance I", distancePID.getI());
      double dD = SmartDashboard.getNumber("Vision/Distance D", distancePID.getD());
      
      // Update target distance if changed
      targetDistance = SmartDashboard.getNumber("Vision/Target Distance", targetDistance);
      
      // Update PID controllers if values changed
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
  
  /**
   * Clamps a value between a minimum and maximum.
   * 
   * @param value The value to clamp
   * @param min The minimum allowed value
   * @param max The maximum allowed value
   * @return The clamped value
   */
  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}