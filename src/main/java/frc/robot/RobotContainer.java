// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Garra.IntakeCmd;
import frc.robot.commands.Garra.MunecaCmd;
import frc.robot.commands.Chasis.AlignToAprilTagCommand;
import frc.robot.commands.Chasis.ArcadeDriveCmd;
import frc.robot.commands.Chasis.Drive;
import frc.robot.commands.Chasis.RotarDerecha;
import frc.robot.commands.Chasis.RotarIzquierda;
import frc.robot.commands.Chasis.AutonomoAvanzar;
import frc.robot.commands.Elevador.ResetEncoders;
import frc.robot.commands.Elevador.ElevatorCmd;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Muneca;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * ROBOT CONTAINER
 * Aquí se declaran y conectan todos los subsistemas, comandos y controles del robot.
 * Se añadió la configuración del SendableChooser para seleccionar el comando autónomo.
 */
public class RobotContainer {
  // Subsistemas principales
  private final Chasis chasis = new Chasis();
  private final Elevator elevator = new Elevator();
  private final Vision vision = new Vision();
  private final Muneca muneca = new Muneca();
  private final Intake intake = new Intake();

  // Controles (Xbox Controllers)
  private final XboxController control_1 = new XboxController(1);
  private final XboxController control_2 = new XboxController(2);

  // Limit switches para el elevador
  private final DigitalInput downlimitswitch = new DigitalInput(0);
  private final DigitalInput uplimitswitch = new DigitalInput(1);

  // Comandos reutilizables
  private final Command alignToAprilTagCmd = new AlignToAprilTagCommand(vision, chasis);

  // Selector de comandos para autónomo
  private final SendableChooser<Command> mChooser = new SendableChooser<>();

  // SlewRateLimiter para suavizar aceleraciones del chasis
  private final SlewRateLimiter filter = new SlewRateLimiter(0.2);

  // Constantes de operación
  private static final double ELEVATOR_RAISED_THRESHOLD = 100; // Umbral de altura de elevador
  private static final double SLOW_FACTOR = 0.5; // Reducción de velocidad si elevador está arriba

  /**
   * Constructor: inicializa la estructura del robot y configura comandos/bindings.
   */
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureAutonomousOptions();
  }

  /**
   * Configura los comandos por defecto de cada subsistema.
   * - Chasis: ArcadeDriveCmd con lógica de reducción de velocidad si elevador está arriba.
   * - Muneca: comando por defecto que lee joystick 2.
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
        double turn = control_1.getRawAxis(4);
        if (elevator.getCurrentPosition() > ELEVATOR_RAISED_THRESHOLD) {
          turn *= SLOW_FACTOR;
        }
        return turn * 0.4;
      }
    ));

    muneca.setDefaultCommand(new MunecaCmd(
      muneca,
      () -> control_2.getRawAxis(1) * 0.5 // Ajusta el 0.5 para la velocidad máxima deseada
    ));
  }

  /**
   * Configura las opciones disponibles para el modo autónomo y publica el chooser en el Dashboard.
   * Añade como opción por defecto el comando AutonomoAvanzar que simplemente avanza hacia adelante.
   */
  private void configureAutonomousOptions() {
    // Comandos de ejemplo para autónomo
    Command autoAdvance = new AutonomoAvanzar(chasis, 2.0, 0.5); // Avanza 2 metros al 50%
    Command drive1m = new Drive(chasis, 2.0, 0.5); // Avanza 1 metro al 50%
   

    // Registrar opciones en el chooser
    mChooser.setDefaultOption("Avanzar 2m (AutonomoAvanzar)", autoAdvance);
  
    // Publicar en SmartDashboard para selección desde el driver station
    SmartDashboard.putData("Autonomous Mode", mChooser);
  }

  /**
   * Configura los bindings entre botones físicos y comandos.
   * - Botón 3: alineación continua con AprilTag.
   * - Botón 1: subir elevador (mientras se mantenga presionado).
   * - Botón 2: bajar elevador (mientras se mantenga presionado).
   * - Botón 4: reset encoder del elevador.
   * - Control 2 botones para intake.
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
    
    // Controlador 2 - Operador
    new JoystickButton(control_2, 5)
      .whileTrue(new IntakeCmd(intake, 0.8)); // Succionar

    new JoystickButton(control_2, 6)
      .whileTrue(new IntakeCmd(intake, -0.8)); // Expulsar
  }

  /**
   * Devuelve el comando autónomo seleccionado para ser ejecutado por el robot.
   * Si no se ha seleccionado nada, devolverá la opción por defecto registrada en el chooser.
   */
  public Command getAutonomousCommand() {
    return mChooser.getSelected();
  }
}