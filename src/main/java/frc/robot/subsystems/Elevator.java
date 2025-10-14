package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorFalcon = new TalonFX(2); // Falcon con ID 2

  private double kP = 0.1;
  private double kI = 0;
  private double kD = 0;

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Cambia si tu motor está al revés
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;
    config.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(40.0)
      .withStatorCurrentLimitEnable(true);

    elevatorFalcon.getConfigurator().apply(config);

    // Botón de reset encoder en Dashboard
    SmartDashboard.putBoolean("Elevator/Reset Encoder", false);
  }

  public void set_speed(double speed) {
    elevatorFalcon.set(speed);
  }

  public double getCurrentPosition() {
    // Phoenix6: getPosition() retorna Angle, usa getValueAsDouble() (rotaciones)
    return elevatorFalcon.getPosition().getValueAsDouble();
  }

  public void reset_encoders() {
    elevatorFalcon.setPosition(0);
  }

  public void ElevatorGoPosition(double targetRotations) {
    elevatorFalcon.setControl(new PositionVoltage(targetRotations));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Falcon Position", getCurrentPosition());

    // Botón para resetear el encoder desde el dashboard
    if (SmartDashboard.getBoolean("Elevator/Reset Encoder", false)) {
      reset_encoders();
      SmartDashboard.putBoolean("Elevator/Reset Encoder", false); // Para que sea pulsador
    }
  }
}