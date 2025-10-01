// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Garra extends SubsystemBase {
  private final TalonFX falcon_garra = new TalonFX(2); //tubos
  private final SparkMax redline_gara = new SparkMax(3, MotorType.kBrushless); //pelotas
  private final SparkMaxConfig redline_config = new SparkMaxConfig();
  

  // private final TalonFXConfiguration falcon_config = new TalonFXConfiguration();
  // private final CurrentLimitsConfigs limits_Configs = new CurrentLimitsConfigs();

  /** Creates a new Garra. */
  public Garra() {
    // Configuración para Falcon 500 (TalonFX)
    TalonFXConfiguration falconConfig = new TalonFXConfiguration();
    falconConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40.0)
        .withStatorCurrentLimitEnable(true);

    falcon_garra.getConfigurator().apply(falconConfig.withCurrentLimits(currentLimits));

    redline_config.inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    redline_gara.configure(redline_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void set_motor_garra(double speed, double speed1){
    falcon_garra.set(speed);
    redline_gara.set(speed1);
    // redline_gara.set(speed);
  }



  @Override
  public void periodic() {
    
  }
}
