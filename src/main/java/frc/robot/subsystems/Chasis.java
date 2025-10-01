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
 * Subsistema que controla el chasis del robot, incluyendo la tracción diferencial
 * y los encoders para el control de movimiento.
 */
public class Chasis extends SubsystemBase {
  // Motores
  private final SparkMax leftFrontMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftBackMotor = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax rightFrontMotor = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax rightBackMotor = new SparkMax(5, MotorType.kBrushless);

  // Configuraciones para los motores
  private final SparkMaxConfig baseConfig = new SparkMaxConfig();
  private final SparkMaxConfig leftFrontConfig = new SparkMaxConfig();
  private final SparkMaxConfig leftBackConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightBackConfig = new SparkMaxConfig();

  // Controladores PID y encoders
  private final SparkClosedLoopController pidLeftMotors = leftFrontMotor.getClosedLoopController();
  private final SparkClosedLoopController pidRightMotors = rightFrontMotor.getClosedLoopController();
  private final RelativeEncoder leftEncoder, rightEncoder;

  // Constantes del sistema
  private static final double WHEEL_DIAMETER = 0.1524; // 6 pulgadas en metros
  private static final double GEAR_RATIO = 8.45;
  private static final double ROBOT_DIAMETER = 0.5461; // Diámetro del robot en metros

  // Constantes del PID
  private static final double kP = 0.00001;
  private static final double kI = 0;
  private static final double kD = 0;

  // Controlador PID perfilado para movimiento lineal
  private final ProfiledPIDController linearController;
  private static final double MAX_VELOCITY = 2.0; // m/s
  private static final double MAX_ACCELERATION = 1.0; // m/s²

  /**
   * Constructor del subsistema Chasis.
   */
  public Chasis() {
    // Configuración base para todos los motores
    baseConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    baseConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    baseConfig.closedLoop.pid(kP, kI, kD).outputRange(-0.4, 0.4);

    // Aplicar configuraciones específicas
    leftFrontConfig.apply(baseConfig);
    leftBackConfig.apply(baseConfig).follow(leftFrontMotor);
    rightFrontConfig.apply(baseConfig).inverted(true);
    rightBackConfig.apply(baseConfig).follow(rightFrontMotor).inverted(true);

    // Configurar los motores
    leftFrontMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftBackMotor.configure(leftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFrontMotor.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightBackMotor.configure(rightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Obtener los encoders
    leftEncoder = leftFrontMotor.getEncoder();
    rightEncoder = rightFrontMotor.getEncoder();
    resetEncoder();
    
    // Configurar el controlador PID perfilado
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    linearController = new ProfiledPIDController(kP, kI, kD, constraints);
    linearController.setTolerance(0.1); // 10 cm de tolerancia
  }

  /**
   * Establece la velocidad de los motores del chasis.
   * @param leftSpeed Velocidad del lado izquierdo (-1 a 1)
   * @param rightSpeed Velocidad del lado derecho (-1 a 1)
   */
  public void set_motors(double leftSpeed, double rightSpeed) {
    leftFrontMotor.set(leftSpeed);
    rightFrontMotor.set(rightSpeed);
  }

  /**
   * Establece una posición objetivo usando PID.
   * @param position Posición objetivo en rotaciones de motor
   */
  public void setPosition(double position) {
    pidLeftMotors.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    pidRightMotors.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /**
   * Establece una velocidad objetivo usando PID.
   * @param velocity Velocidad objetivo en RPM
   */
  public void setVelocity(double velocity) {
    pidRightMotors.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    pidLeftMotors.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  /**
   * Restablece los encoders a cero.
   */
  public void resetEncoder() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * Obtiene la distancia media recorrida en metros.
   * @return Distancia media en metros
   */
  public double getAverageEncoderDistance() {
    double leftDistance = (leftEncoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
    double rightDistance = (rightEncoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
    
    return (leftDistance + rightDistance) / 2.0;
  }

  /**
   * Conduce hasta una distancia objetivo usando PID.
   * @param targetDistance Distancia objetivo en metros
   * @return Error absoluto actual en metros
   */
  public double driveToDistance(double targetDistance) {
    double current = getAverageEncoderDistance();
    double output = linearController.calculate(current, targetDistance);

    // Limitar salida
    output = Math.max(-1.0, Math.min(1.0, output));
    set_motors(output, output);

    return Math.abs(targetDistance - current);
  }

  /**
   * Verifica si el robot ha alcanzado el punto objetivo.
   * @return true si está en el punto objetivo
   */
  public boolean atSetpoint() {
    return linearController.atSetpoint();
  }

  /**
   * Calcula la distancia de rotación para un ángulo dado.
   * @param degree Ángulo en grados
   * @return Distancia que deben recorrer las ruedas para girar el ángulo especificado
   */
  public double rotacion(double degree) {
    return (Math.PI * ROBOT_DIAMETER) / (degree);
  }

  /**
   * Obtiene la distancia recorrida por el encoder izquierdo en metros.
   * @return Distancia en metros
   */
  public double get_left_encoder_distance() {
    return (leftEncoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
  }

  /**
   * Obtiene la distancia recorrida por el encoder derecho en metros.
   * @return Distancia en metros
   */
  public double get_right_encoder_distance() {
    return (rightEncoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
  }

  @Override
  public void periodic() {
    // Publicar datos en el Dashboard
    SmartDashboard.putNumber("Chasis/Posición Encoder Derecho", rightEncoder.getPosition());
    SmartDashboard.putNumber("Chasis/Posición Encoder Izquierdo", leftEncoder.getPosition());
    SmartDashboard.putNumber("Chasis/Distancia Recorrida (m)", getAverageEncoderDistance());
    SmartDashboard.putNumber("Chasis/Encoder Izquierdo (m)", get_left_encoder_distance());
    SmartDashboard.putNumber("Chasis/Encoder Derecho (m)", get_right_encoder_distance());

    // Botón para restablecer encoders desde el Dashboard
    if (SmartDashboard.getBoolean("Chasis/Restablecer Encoders", false)) {
      SmartDashboard.putBoolean("Chasis/Restablecer Encoders", false);
      resetEncoder();
    }
  }
}