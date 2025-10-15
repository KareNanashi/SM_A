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
 * SUBSISTEMA: Muneca
 * Controla el motor NEO de la muñeca de la garra (ID 32) usando la API experimental de REV.
 * Permite movimiento en velocidad directa (-1 a 1), inicializa en 0.
 */
public class Muneca extends SubsystemBase {
  private final SparkMax munecaMotor = new SparkMax(32, MotorType.kBrushless);
  private final SparkMaxConfig munecaConfig = new SparkMaxConfig();

  public Muneca() {
    // Configuración: modo brake, límite de corriente
    munecaConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    munecaMotor.configure(munecaConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    munecaMotor.set(0); // Garantiza que al iniciar, el motor está en cero
    SmartDashboard.putNumber("Muneca/Velocidad", 0);
  }

  /** Control directo de velocidad (-1 a 1). */
  public void setSpeed(double speed) {
    munecaMotor.set(speed);
    SmartDashboard.putNumber("Muneca/Velocidad", speed);
  }
}