package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SUBSISTEMA: Intake
 * Controla dos motores NEO (ID 4 invertido y 10 normal) para succionar una pelota.
 */
public class Intake extends SubsystemBase {
  private final SparkMax intakeMotorA = new SparkMax(4, MotorType.kBrushless); // Invertido
  private final SparkMax intakeMotorB = new SparkMax(10, MotorType.kBrushless);

  public Intake() {
    SparkMaxConfig configA = new SparkMaxConfig();
    SparkMaxConfig configB = new SparkMaxConfig();

    configA.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true); // Invertir ID 4
    configB.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    intakeMotorA.configure(configA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorB.configure(configB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeMotorA.set(0);
    intakeMotorB.set(0);

    SmartDashboard.putNumber("Intake/Velocidad", 0);
  }

  /** Control directo de velocidad (-1 a 1) para ambos motores. */
  public void setSpeed(double speed) {
    intakeMotorA.set(speed);
    intakeMotorB.set(speed);
    SmartDashboard.putNumber("Intake/Velocidad", speed);
  }

  /** Detiene ambos motores. */
  public void stop() {
    setSpeed(0);
  }
}