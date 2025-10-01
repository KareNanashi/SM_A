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

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final SparkMax Intake_1 = new SparkMax(10, MotorType.kBrushless);
  private final SparkMaxConfig intake1config = new SparkMaxConfig();
  private final SparkClosedLoopController IntakePID1 = Intake_1.getClosedLoopController();
  private final RelativeEncoder Intake_encoder_1;

  private final double POSITION_UP = -116;
  private final double POSITION_DOWN = 120;

  // Constantes del sistema
  // private static final double GEAR_RATIO = 125; // 3 etapas de 5:1 (5*5*5 = 125)
  // private static final double SPROCKET_CIRCUMFERENCE = (0.05 * Math.PI); // Circunferencia del sprocket en metros
  // private static final double METERS_TO_ROTATIONS = GEAR_RATIO / SPROCKET_CIRCUMFERENCE;

  // Constantes del PID
  private double kP = 0.1;
  private double kI = 0;
  private double kD = 0;

  // Controlador PID perfilado


  public Intake() {
    // Configuración del motor
    intake1config.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);
    intake1config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    intake1config.closedLoop.pid(kP, kI, kD).outputRange(-0.8, 0.8);
    Intake_1.configure(intake1config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Obtener el encoder
    Intake_encoder_1 = Intake_1.getEncoder();

    
  }

  // Método para establecer la velocidad del elevador
  public void set_speed(double speed) {
    Intake_1.set(speed);
  }

  // Método para obtener la posición actual del elevador en metros
  public double getCurrentPosition() {
    return Intake_encoder_1.getPosition();
  }


  public void reset_encoders(){
    Intake_encoder_1.setPosition(0);
  }


  public void goPosition(double target){
    IntakePID1.setReference(target,ControlType.kPosition);
  }

  public void positionDOWN(){
    goPosition(POSITION_DOWN);
  }
  public void positionUP(){
    goPosition(POSITION_UP);
  }


  @Override
  public void periodic() {
    // Mostrar la posición del elevador en SmartDashboard
    SmartDashboard.putNumber("Intake Position (m)", getCurrentPosition());

  }
}
