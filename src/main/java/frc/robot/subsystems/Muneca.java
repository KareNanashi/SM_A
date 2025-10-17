// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SUBSISTEMA: Muneca con HOLD
 * Controla el motor NEO de la muñeca (ID 32) con modo brake y mantiene posición cuando el joystick está suelto.
 */
public class Muneca extends SubsystemBase {
  private final SparkMax munecaMotor = new SparkMax(32, MotorType.kBrushless);
  private final SparkMaxConfig munecaConfig = new SparkMaxConfig();

  // Encoder y controlador PID del SparkMax
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pidController;
  private double holdPosition = 0.0;

  // PID experimental (ajusta según tu sistema)
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Zona muerta para antidrift
  private static final double DEADZONE = 0.07;

  public Muneca() {
    // Configuración: modo brake, límite de corriente
    munecaConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);
    munecaConfig.closedLoop.pid(kP, kI, kD).outputRange(-0.5, 0.5);
    munecaMotor.configure(munecaConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = munecaMotor.getEncoder();
    pidController = munecaMotor.getClosedLoopController();

    // Inicializa posición y motor en cero
    encoder.setPosition(0);
    holdPosition = 0;
    munecaMotor.set(0);
    SmartDashboard.putNumber("Muneca/Velocidad", 0);
    SmartDashboard.putNumber("Muneca/Posicion", 0);
    SmartDashboard.putBoolean("Muneca/HoldActive", false);
  }

  /**
   * Control directo de velocidad (-1 a 1) o hold si está en zona muerta.
   * Si la entrada es menor que la zona muerta, activa hold en la posición actual.
   */
  public void setSpeed(double speed) {
    if (Math.abs(speed) < DEADZONE) {
      // HOLD: mantener posición con    bbn  PID cuando joystick está suelto
      pidController.setReference(holdPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      SmartDashboard.putBoolean("Muneca/HoldActive", true);
    } else {
      // Control manual: velocidad directa
      munecaMotor.set(speed);
      holdPosition = encoder.getPosition(); // Actualiza posición para hold
      SmartDashboard.putBoolean("Muneca/HoldActive", false);
    }
    SmartDashboard.putNumber("Muneca/Velocidad", speed);
    SmartDashboard.putNumber("Muneca/Posicion", encoder.getPosition());
  }
}