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
 * SUBSISTEMA: VISIÓN
 * Maneja detección y seguimiento de AprilTags usando PhotonVision.
 * Proporciona información de posición, ángulo y ID de tag.
 * Métodos clave:
 * - periodic: Actualiza y publica datos de visión en el Dashboard.
 * - getXAxis/getYAxis/getZAngle/getIDApriltag: Devuelven datos del target actual.
 * - hasTarget: Indica si hay tag detectado.
 * - isValidTag: Filtra tags de interés según IDs configurados.
 */
public class Vision extends SubsystemBase {
  // Cámara PhotonVision configurada con nombre "photonvision"
  private final PhotonCamera camera;
  
  // Variables de posición y estado del target
  private double xAxis = 0.0;
  private double yAxis = 0.0;
  private double zAngle = 0.0;
  private int tagId = -1;
  private boolean hasTarget = false;
  
  // Lista de IDs válidos para filtrar tags de interés
  private final List<Integer> validTagIds = List.of(1, 2, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22);
  
  /** Constructor: inicializa cámara y datos por defecto. */
  public Vision() {
    camera = new PhotonCamera("photonvision");
    resetTargetData();
  }
  
  /** Reinicia los datos del target detectado. */
  public void resetTargetData() {
    xAxis = 0.0;
    yAxis = 0.0;
    zAngle = 0.0;
    tagId = -1;
    hasTarget = false;
  }

  /**
   * Se llama automáticamente cada ciclo: actualiza datos de visión y publica en Dashboard.
   */
  @Override
  public void periodic() {
    updateVisionData();
    displayDataOnDashboard();
  }

  /**
   * Actualiza los datos de visión desde la PhotonCamera.
   */
  private void updateVisionData() {
    PhotonPipelineResult result = camera.getLatestResult();
    
    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      
      // Actualiza datos de posición y ángulo
      Transform3d targetTransform = bestTarget.getBestCameraToTarget();
      xAxis = targetTransform.getX();
      yAxis = targetTransform.getY();
      zAngle = bestTarget.getYaw();
      tagId = bestTarget.getFiducialId();
      hasTarget = true;
      
      // Log para debugging
      System.out.println("AprilTag detected - ID: " + tagId + ", Distance: " + xAxis + "m, Angle: " + zAngle + "°");
    } else {
      // Si no hay targets, indica que no hay detección
      hasTarget = false;
      if (DriverStation.isEnabled()) {
        System.out.println("No AprilTags detected");
      }
    }
  }
  
  /**
   * Muestra los datos de visión en SmartDashboard para depuración y monitoreo.
   */
  private void displayDataOnDashboard() {
    SmartDashboard.putBoolean("Vision/Target Detected", hasTarget);
    SmartDashboard.putNumber("Vision/AprilTag ID", tagId);
    SmartDashboard.putNumber("Vision/Distance (m)", xAxis);
    SmartDashboard.putNumber("Vision/Angle (deg)", zAngle);
    SmartDashboard.putBoolean("Vision/Valid Tag", isValidTag());
  }

  /** 
   * Devuelve true si el tag detectado está en la lista de tags válidos. 
   */
  public boolean isValidTag() {
    return hasTarget && validTagIds.contains(tagId);
  }
  
  /** Devuelve la posición X relativa al tag en metros. */
  public double getXAxis() {
    return xAxis;
  }
  
  /** Devuelve la posición Y relativa al tag en metros. */
  public double getYAxis() {
    return yAxis;
  }
  
  /** Devuelve el ángulo yaw (Z) relativo al tag en grados. */
  public double getZAngle() {
    return zAngle;
  }
  
  /** Devuelve el ID del AprilTag actual, o -1 si no hay detección. */
  public int getIDApriltag() {
    return tagId;
  }
  
  /** Devuelve true si hay target detectado por la cámara. */
  public boolean hasTarget() {
    return hasTarget;
  }
}