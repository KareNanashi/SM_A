// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax elevator_1 = new SparkMax(4, MotorType.kBrushless);
  private final SparkMaxConfig elev1config = new SparkMaxConfig();
  private final SparkClosedLoopController elevatorPID1 = elevator_1.getClosedLoopController();
  private final RelativeEncoder elevator_encoder_1;

  // Constantes del sistema
  // private static final double GEAR_RATIO = 125; // 3 etapas de 5:1 (5*5*5 = 125)
  // private static final double SPROCKET_CIRCUMFERENCE = (0.05 * Math.PI); // Circunferencia del sprocket en metros
  // private static final double METERS_TO_ROTATIONS = GEAR_RATIO / SPROCKET_CIRCUMFERENCE;

  // Constantes del PID
  private double kP = 0.1;
  private double kI = 0;
  private double kD = 0;

  // Controlador PID perfilado


  public Elevator() {
    // Configuración del motor
    elev1config.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
    elev1config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    elev1config.closedLoop.pid(kP, kI, kD).outputRange(-0.75, 0.75);
    elevator_1.configure(elev1config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Obtener el encoder
    elevator_encoder_1 = elevator_1.getEncoder();

    
  }

  // Método para establecer la posición del elevador en metros
  // public void set_position(double targetPositionMetros) {
  //   double targetPositionRotations = targetPositionMetros * METERS_TO_ROTATIONS;
  //   elevatorPID1.setReference(targetPositionRotations, SparkMax.ControlType.kPosition);
  // }

  // Método para establecer la velocidad del elevador
  public void set_speed(double speed) {
    elevator_1.set(-speed);
  }

  // Método para obtener la posición actual del elevador en metros
  public double getCurrentPosition() {
    return elevator_encoder_1.getPosition();
  }

  // Método para mostrar la posición del encoder en SmartDashboard
  public void position_encoders() {
    SmartDashboard.putNumber("Elevator1", elevator_encoder_1.getPosition());
  }

  public void reset_encoders(){
    elevator_encoder_1.setPosition(0);
  }

  public void ElevatorGoPosition(double target){
    elevatorPID1.setReference(target, ControlType.kPosition);
  }


  @Override
  public void periodic() {
    // Mostrar la posición del elevador en SmartDashboard
    SmartDashboard.putNumber("Elevator Position (m)", getCurrentPosition());

    position_encoders();
  }
}
