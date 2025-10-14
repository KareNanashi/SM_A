// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SUBSISTEMA: CHASIS 
 * Controla la tracción diferencial del robot, maneja los motores principales y los encoders para el movimiento.
 * Métodos clave:
 * - set_motors: Control directo de velocidad de motores.
 * - setPosition/setVelocity: Control en lazo cerrado (PID) para posición/velocidad.
 * - resetEncoder: Reinicia los encoders a cero.
 * - getAverageEncoderDistance: Devuelve la distancia media recorrida en metros.
 * - driveToDistance/atSetpoint: PID para mover a una distancia específica.
 * - rotacion: Calcula la distancia que deben recorrer las ruedas para girar un ángulo dado.
 * - get_left_encoder_distance/get_right_encoder_distance: Lectura en metros de cada lado.
 * - periodic: Publica datos en SmartDashboard y permite resetear encoders desde Dashboard.
 */
public class Chasis extends SubsystemBase {
  // Motores principales: IDs configurados según el robot
  private final SparkMax leftFrontMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftBackMotor = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax rightFrontMotor = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax rightBackMotor = new SparkMax(5, MotorType.kBrushless);

  // Configuración base y específica para cada motor
  private final SparkMaxConfig baseConfig = new SparkMaxConfig();
  private final SparkMaxConfig leftFrontConfig = new SparkMaxConfig();
  private final SparkMaxConfig leftBackConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightBackConfig = new SparkMaxConfig();

  // Controladores PID y encoders
  private final SparkClosedLoopController pidLeftMotors = leftFrontMotor.getClosedLoopController();
  private final SparkClosedLoopController pidRightMotors = rightFrontMotor.getClosedLoopController();
  private final RelativeEncoder leftEncoder, rightEncoder;

  // Constantes físicas del robot (ajusta para tu robot real)
  private static final double WHEEL_DIAMETER = 0.1524; // 6 pulgadas en metros
  private static final double GEAR_RATIO = 8.45;
  private static final double ROBOT_DIAMETER = 0.5461; // Diámetro entre ruedas (m)

  // Constantes de PID (ajusta según tu robot)
  private static final double kP = 0.00001;
  private static final double kI = 0;
  private static final double kD = 0;

  // Controlador PID perfilado para trayectorias lineales
  private final ProfiledPIDController linearController;
  private static final double MAX_VELOCITY = 2.0; // m/s (ajusta según tu robot)
  private static final double MAX_ACCELERATION = 1.0; // m/s²

  /**
   * Constructor: configura motores, encoders y controlador PID.
   * También establece parámetros base y publica encoders en el Dashboard.
   */
  public Chasis() {
    // Configuración estándar: modo freno, límite de corriente, conversión de encoders, rango de salida
    baseConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    baseConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    baseConfig.closedLoop.pid(kP, kI, kD).outputRange(-0.4, 0.4);

    // Configura cada motor, siguiendo e invirtiendo según corresponda
    leftFrontConfig.apply(baseConfig);
    leftBackConfig.apply(baseConfig).follow(leftFrontMotor);
    rightFrontConfig.apply(baseConfig).inverted(true);
    rightBackConfig.apply(baseConfig).follow(rightFrontMotor).inverted(true);

    leftFrontMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftBackMotor.configure(leftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFrontMotor.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightBackMotor.configure(rightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Encoders conectados a los motores frontales
    leftEncoder = leftFrontMotor.getEncoder();
    rightEncoder = rightFrontMotor.getEncoder();
    resetEncoder(); // Inicia en cero

    // Controlador PID para movimiento lineal
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    linearController = new ProfiledPIDController(kP, kI, kD, constraints);
    linearController.setTolerance(0.1); // 10 cm de tolerancia
  }

  /**
   * Control directo de motores (manual/arcade/tank).
   */
  public void set_motors(double leftSpeed, double rightSpeed) {
    leftFrontMotor.set(leftSpeed);
    rightFrontMotor.set(rightSpeed);
  }

  /**
   * Control en lazo cerrado (PID) para posición.
   */
  public void setPosition(double position) {
    pidLeftMotors.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    pidRightMotors.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /**
   * Control en lazo cerrado para velocidad.
   */
  public void setVelocity(double velocity) {
    pidRightMotors.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    pidLeftMotors.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  /** Restablece encoders a cero. */
  public void resetEncoder() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * Devuelve la distancia media recorrida (en metros).
   */
  public double getAverageEncoderDistance() {
    double leftDistance = (leftEncoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
    double rightDistance = (rightEncoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
    return (leftDistance + rightDistance) / 2.0;
  }

  /**
   * Hace que el robot avance a una distancia objetivo usando PID.
   * Retorna el error absoluto actual.
   */
  public double driveToDistance(double targetDistance) {
    double current = getAverageEncoderDistance();
    double output = linearController.calculate(current, targetDistance);

    // Limita la salida a [-1, 1]
    output = Math.max(-1.0, Math.min(1.0, output));
    set_motors(output, output);

    return Math.abs(targetDistance - current);
  }

  /** Devuelve verdadero si el robot está en el punto objetivo (según PID) */
  public boolean atSetpoint() {
    return linearController.atSetpoint();
  }

  /**
   * Calcula la distancia que deben recorrer las ruedas para girar X grados sobre su eje.
   * IMPORTANTE: La fórmula actual debe ser revisada si se requiere precisión absoluta de giro.
   */
  public double rotacion(double degree) {
    // FÓRMULA ACTUAL: Debe corregirse si se busca precisión absoluta.
    return (Math.PI * ROBOT_DIAMETER) / (degree);
  }

  /** Lectura de encoder izquierdo en metros. */
  public double get_left_encoder_distance() {
    return (leftEncoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
  }

  /** Lectura de encoder derecho en metros. */
  public double get_right_encoder_distance() {
    return (rightEncoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
  }

  /**
   * Llamado periódicamente: actualiza datos en el Dashboard y permite resetear encoders desde la interfaz.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Chasis/Posición Encoder Derecho", rightEncoder.getPosition());
    SmartDashboard.putNumber("Chasis/Posición Encoder Izquierdo", leftEncoder.getPosition());
    SmartDashboard.putNumber("Chasis/Distancia Recorrida (m)", getAverageEncoderDistance());
    SmartDashboard.putNumber("Chasis/Encoder Izquierdo (m)", get_left_encoder_distance());
    SmartDashboard.putNumber("Chasis/Encoder Derecho (m)", get_right_encoder_distance());

    // Si el usuario pulsa el botón virtual, resetea encoders
    if (SmartDashboard.getBoolean("Chasis/Restablecer Encoders", false)) {
      SmartDashboard.putBoolean("Chasis/Restablecer Encoders", false);
      resetEncoder();
    }
  }
}