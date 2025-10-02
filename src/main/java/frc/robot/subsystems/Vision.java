// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem using PhotonVision for AprilTag detection and tracking.
 */
public class Vision extends SubsystemBase {
  // The PhotonVision camera
  private final PhotonCamera camera;
  
  // Target information
  private double xAxis = 0.0;
  private double yAxis = 0.0;
  private double zAngle = 0.0;
  private int tagId = -1;
  private boolean hasTarget = false;
  
  // For filtered targets
  private final List<Integer> validTagIds = List.of(1, 2, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22);
  
  /**
   * Creates a new Vision subsystem.
   */
  public Vision() {
    camera = new PhotonCamera("photonvision_Port_1184_Output_MJPEG_Server");
    resetTargetData();
  }
  
  /**
   * Reset all target data to default values.
   */
  public void resetTargetData() {
    xAxis = 0.0;
    yAxis = 0.0;
    zAngle = 0.0;
    tagId = -1;
    hasTarget = false;
  }

  @Override
  public void periodic() {
    updateVisionData();
    displayDataOnDashboard();
  }

  /**
   * Update vision data from the PhotonVision camera.
   */
  private void updateVisionData() {
    PhotonPipelineResult result = camera.getLatestResult();
    
    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      
      // Update position data
      Transform3d targetTransform = bestTarget.getBestCameraToTarget();
      xAxis = targetTransform.getX();
      yAxis = targetTransform.getY();
      zAngle = bestTarget.getYaw();
      tagId = bestTarget.getFiducialId();
      hasTarget = true;
      
      // Log successful detection
      System.out.println("AprilTag detected - ID: " + tagId + ", Distance: " + xAxis + "m, Angle: " + zAngle + "°");
    } else {
      // No targets found
      hasTarget = false;
      if (DriverStation.isEnabled()) {
        System.out.println("No AprilTags detected");
      }
    }
  }
  
  /**
   * Display vision data on the SmartDashboard.
   */
  private void displayDataOnDashboard() {
    SmartDashboard.putBoolean("Vision/Target Detected", hasTarget);
    SmartDashboard.putNumber("Vision/AprilTag ID", tagId);
    SmartDashboard.putNumber("Vision/Distance (m)", xAxis);
    SmartDashboard.putNumber("Vision/Angle (deg)", zAngle);
    SmartDashboard.putBoolean("Vision/Valid Tag", isValidTag());
  }

  /**
   * Check if the currently detected AprilTag is considered valid.
   * 
   * @return true if the current tag is valid, false otherwise
   */
  public boolean isValidTag() {
    return hasTarget && validTagIds.contains(tagId);
  }
  
  /**
   * Get the X-axis position relative to the target in meters.
   * 
   * @return X position in meters (positive = forward)
   */
  public double getXAxis() {
    return xAxis;
  }
  
  /**
   * Get the Y-axis position relative to the target in meters.
   * 
   * @return Y position in meters (positive = right)
   */
  public double getYAxis() {
    return yAxis;
  }
  
  /**
   * Get the Z-axis rotation (yaw) relative to the target in degrees.
   * 
   * @return Z rotation in degrees (positive = clockwise)
   */
  public double getZAngle() {
    return zAngle;
  }
  
  /**
   * Get the ID of the currently tracked AprilTag.
   * 
   * @return AprilTag ID, or -1 if no tag is detected
   */
  public int getIDApriltag() {
    return tagId;
  }
  
  /**
   * Check if the camera has a valid target.
   * 
   * @return true if a target is being tracked, false otherwise
   */
  public boolean hasTarget() {
    return hasTarget;
  }
}