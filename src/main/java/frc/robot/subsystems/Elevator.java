// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SUBSISTEMA: ELEVADOR
 * Controla el motor Falcon para subir/bajar el elevador usando setpoint, velocidad directa y lectura de posición.
 * Métodos clave:
 * - set_speed: Control directo (-1 a 1).
 * - getCurrentPosition: Lectura de posición en rotaciones.
 * - reset_encoders: Reinicia el encoder del Falcon.
 * - ElevatorGoPosition: Control de posición cerrada usando voltaje (Phoenix6).
 * - periodic: Actualiza datos en SmartDashboard y permite reset manual desde dashboard.
 */
public class Elevator extends SubsystemBase {
  private final TalonFX elevatorFalcon = new TalonFX(2); // Falcon con ID 2

  // Parámetros PID (ajusta para tu sistema)
  private double kP = 0.1;
  private double kI = 0;
  private double kD = 0;

  // Última posición registrada para mantener con PID
  private double holdPosition = 0;

  /**
   * Constructor: configura el Falcon, PID e inversion.
   */
  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;
    config.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(40.0)
      .withStatorCurrentLimitEnable(true);

    elevatorFalcon.getConfigurator().apply(config);

    // Activa modo freno (brake)
    elevatorFalcon.setNeutralMode(NeutralModeValue.Brake);

    // Botón de reset encoder en Dashboard (toggle)
    SmartDashboard.putBoolean("Elevator/Reset Encoder", false);
  }

  /**
   * Control directo de velocidad (-1 a 1).
   */
  public void set_speed(double speed) {
    if (speed == 0) {
      // Mantener posición actual usando PID si no hay comando de velocidad
      holdPosition = getCurrentPosition();
      ElevatorGoPosition(holdPosition);
    } else {
      elevatorFalcon.set(speed);
      holdPosition = getCurrentPosition();
    }
  }

  /**
   * Devuelve la posición actual del Falcon en rotaciones (Phoenix6).
   */
  public double getCurrentPosition() {
    return elevatorFalcon.getPosition().getValueAsDouble();
  }

  /** Resetea el encoder del Falcon a cero. */
  public void reset_encoders() {
    elevatorFalcon.setPosition(0);
    holdPosition = 0;
  }

  /**
   * Control de posición cerrada usando voltaje.
   * @param targetRotations Número de rotaciones objetivo.
   */
  public void ElevatorGoPosition(double targetRotations) {
    elevatorFalcon.setControl(new PositionVoltage(targetRotations));
  }

  /**
   * Llamado periódicamente: publica posición en Dashboard, permite reset manual desde interfaz.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Falcon Position", getCurrentPosition());

    // Si el usuario pulsa el botón virtual, resetea encoder
    if (SmartDashboard.getBoolean("Elevator/Reset Encoder", false)) {
      reset_encoders();
      SmartDashboard.putBoolean("Elevator/Reset Encoder", false); // Reinicia el pulsador
    }
  }
}