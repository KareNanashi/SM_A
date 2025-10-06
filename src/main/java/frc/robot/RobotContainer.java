// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autonomo.AlignToAprilTagCommand;
import frc.robot.commands.Chasis.ArcadeDriveCmd;
import frc.robot.commands.Chasis.Drive;
import frc.robot.commands.Chasis.RotarDerecha;
import frc.robot.commands.Chasis.RotarIzquierda;
import frc.robot.commands.Elevador.ElevatorCmd;
import frc.robot.commands.Elevador.ElevatorIntakeSequence;
import frc.robot.commands.Elevador.MoveElevatorToPosition;
import frc.robot.commands.Elevador.ResetEncoders;
import frc.robot.commands.Garra.GarraCmd;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.IntakeResetEncoders;
import frc.robot.commands.Intake.IntakeSetPositionCmd;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Garra;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chasis chasis = new Chasis();
  private final Elevator elevator = new Elevator();
  private final Garra garra = new Garra();
  private final Intake intake = new Intake();
  private final Vision vision = new Vision();
  private final XboxController control_1 = new XboxController(0);
  private final XboxController control_2 = new XboxController(1);
  
  // Limit switches
  private final DigitalInput downlimitswitch = new DigitalInput(0);
  private final DigitalInput uplimitswitch = new DigitalInput(1);
  
  // Autonomous commands
  private final Command AlignToAprilTagCommand = new AlignToAprilTagCommand(vision, chasis);
  
  // Command selector for autonomous
  private final SendableChooser<Command> mChooser = new SendableChooser<>();
  
  // Rate limiter for smooth driving
  private final SlewRateLimiter filter = new SlewRateLimiter(0.5);
  
  // Constants
  private static final double ELEVATOR_RAISED_THRESHOLD = 100; // Ajusta este valor según tus mediciones
  private static final double SLOW_FACTOR = 0.5; // Factor para reducir la velocidad cuando el elevador está elevado
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
    
    // Configure default commands
    configureDefaultCommands();
    
    // Configure autonomous options
    configureAutonomousOptions();
    
    // Add CommandScheduler to SmartDashboard
    SmartDashboard.putData(CommandScheduler.getInstance());
  }
  
  /**
   * Configure the default commands for subsystems.
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
        return turn*0.4;
      }
    ));
    
    intake.setDefaultCommand(new IntakeCmd(intake, () -> (-control_2.getRawAxis(1)*0.4)));
  }
  
  /**
   * Configure autonomous options for the sendable chooser.
   */
  private void configureAutonomousOptions() {
    // Set continuous AprilTag following as the default and only autonomous option
    mChooser.setDefaultOption("AprilTag Following", AlignToAprilTagCommand);
    
    // Publicar el chooser en SmartDashboard
    SmartDashboard.putData("Auto Mode", mChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // Controller 1 - Driver
    configureDriverControls();
    
    // Controller 2 - Operator
    configureOperatorControls();
  }
  
  /**
   * Configure the controls for the driver (controller 1).
   */
  private void configureDriverControls() {
    // Elevator control
    new JoystickButton(control_1, 1)
        .whileTrue(new ElevatorCmd(elevator, 0.75)
        .unless(() -> downlimitswitch.get() == false));
    
    new JoystickButton(control_1, 2)
        .whileTrue(new ElevatorCmd(elevator, -0.75)
        .unless(() -> uplimitswitch.get() == false));
    
    // Add vision alignment button - change to continuous following for testing
    new JoystickButton(control_1, 3)
        .whileTrue(new AlignToAprilTagCommand(vision, chasis));
  }
  
  /**
   * Configure the controls for the operator (controller 2).
   */
  private void configureOperatorControls() {
    // Garra controls
    new JoystickButton(control_2, 5)
        .whileTrue(new GarraCmd(garra, -0.5, -0.6))
        .whileFalse(new GarraCmd(garra, 0, 0));
    
    new JoystickButton(control_2, 6)
        .whileTrue(new GarraCmd(garra, 0.5, 0.6))
        .whileFalse(new GarraCmd(garra, 0, 0));
    
    // Elevator position presets
    new Trigger(() -> control_2.getRawButton(3))
        .onTrue(new MoveElevatorToPosition(elevator, 260));
    
    new Trigger(() -> control_2.getRawButton(2))
        .onTrue(new MoveElevatorToPosition(elevator, 0));
    
    new Trigger(() -> control_2.getRawButton(1))
        .onTrue(new MoveElevatorToPosition(elevator, 110));
    
    new Trigger(() -> control_2.getRawButton(4))
        .onTrue(new MoveElevatorToPosition(elevator, 331));
    
    // Reset encoders
    new JoystickButton(control_2, 10).onTrue(new ResetEncoders(elevator));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the selected command from the chooser
    return mChooser.getSelected();
  }
}