// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Garra.IntakeCmd;
import frc.robot.commands.Garra.MunecaCmd;
import frc.robot.subsystems.Muneca;
import frc.robot.commands.Elevador.ResetEncoders;
import frc.robot.commands.Chasis.AlignToAprilTagCommand;
import frc.robot.commands.Chasis.ArcadeDriveCmd;
import frc.robot.commands.Chasis.Drive;
import frc.robot.commands.Chasis.RotarDerecha;
import frc.robot.commands.Chasis.RotarIzquierda;
import frc.robot.commands.Elevador.ElevatorCmd;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * ROBOT CONTAINER
 * Aquí se declaran y conectan todos los subsistemas, comandos y controles del robot.
 * Estructura principal:
 * - Subsistemas: Chasis, Elevator, Vision.
 * - Controles: Dos XboxController.
 * - Limit switches: Para el elevador.
 * - Comandos: ArcadeDrive, AlignToAprilTag, ElevatorCmd, ResetEncoders, entre otros.
 * - SendableChooser: Para seleccionar el modo autónomo.
 * - Métodos clave: configureDefaultCommands, configureAutonomousOptions, configureBindings.
 */
public class RobotContainer {
  // Subsistemas principales
  private final Chasis chasis = new Chasis();
  private final Elevator elevator = new Elevator();
  private final Vision vision = new Vision();

  // Controles (Xbox Controllers)
  private final XboxController control_1 = new XboxController(1);
  private final XboxController control_2 = new XboxController(0);

  // Limit switches para el elevador
  private final DigitalInput downlimitswitch = new DigitalInput(0);
  private final DigitalInput uplimitswitch = new DigitalInput(1);

  // Comando autónomo principal: seguir AprilTag
  private final Command AlignToAprilTagCommand = new AlignToAprilTagCommand(vision, chasis);
  private final Intake intake = new Intake();
  // Selector de comandos para autónomo
  private final SendableChooser<Command> mChooser = new SendableChooser<>();

  // SlewRateLimiter para suavizar aceleraciones del chasis
  private final SlewRateLimiter filter = new SlewRateLimiter(0.5);

  // Constantes de operación
  private static final double ELEVATOR_RAISED_THRESHOLD = 100; // Umbral de altura de elevador
  private static final double SLOW_FACTOR = 0.5; // Reducción de velocidad si elevador está arriba
  private final Muneca muneca = new Muneca();
  /**
   * Constructor: inicializa la estructura del robot.
   * IMPORTANTE: Se deben llamar los métodos de configuración aquí.
   */
  public RobotContainer() {
    configureDefaultCommands();
    configureAutonomousOptions();
    configureBindings();
    // Puedes agregar aquí otros inicializadores si tienes subsistemas extra.
  }

  /**
   * Configura los comandos por defecto de cada subsistema.
   * - Chasis: ArcadeDriveCmd con lógica de reducción de velocidad si elevador está arriba.
   */
  private void configureDefaultCommands() {
    chasis.setDefaultCommand(new ArcadeDriveCmd(
      chasis,
      () -> {
        double speed = -control_1.getRawAxis(1);        
        // Si el elevador está por encima del umbral, reduce la velocidad
        if (elevator.getCurrentPosition() > ELEVATOR_RAISED_THRESHOLD) {
          speed *= SLOW_FACTOR; 
        }
        return filter.calculate(speed);
      },
      () -> {
        double turn = control_1.getRawAxis(2);
        if (elevator.getCurrentPosition() > ELEVATOR_RAISED_THRESHOLD) {
          turn *= SLOW_FACTOR;
        }
        return turn*0.4;
      }
    ));
muneca.setDefaultCommand(new MunecaCmd(
  muneca,
  () -> control_1.getRawAxis(3) * 0.5 // Ajusta el 0.5 para la velocidad máxima deseada
));
  }

  /**
   * Configura las opciones disponibles para el modo autónomo.
   * Actualmente, sólo hay una opción: seguir el AprilTag.
   */
  private void configureAutonomousOptions() {
    mChooser.setDefaultOption("AprilTag Following", AlignToAprilTagCommand);
    SmartDashboard.putData("Auto Mode", mChooser);
  }

  /**
   * Configura los bindings entre botones físicos y comandos.
   * - Botón 3: alineación continua con AprilTag.
   * - Botón 1: subir elevador (mientras se mantenga presionado).
   * - Botón 2: bajar elevador (mientras se mantenga presionado).
   * - Botón 5: reset encoder del elevador.
   */
  private void configureBindings() {
    // Controlador 1 - Driver
    new JoystickButton(control_1, 3)
        .whileTrue(new AlignToAprilTagCommand(vision, chasis));

    new JoystickButton(control_1, 1)
        .whileTrue(new ElevatorCmd(elevator, 0.75, downlimitswitch, uplimitswitch)); // Subir

    new JoystickButton(control_1, 2)
        .whileTrue(new ElevatorCmd(elevator, -0.75, downlimitswitch, uplimitswitch)); // Bajar
    
    new JoystickButton(control_1, 4).onTrue(new ResetEncoders(elevator));    
    
    new JoystickButton(control_1, 6)
    .whileTrue(new IntakeCmd(intake, 0.8)); // Succionar

    new JoystickButton(control_1, 5)
    .whileTrue(new IntakeCmd(intake, -0.8)); // Escupir (expulsar)
  }

  /**
   * Devuelve el comkkando autónomo seleccionado para ser ejecutado por el robot.
   */
  public Command getAutonomousCommand() {
    return mChooser.getSelected();
  }
}