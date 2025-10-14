// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Elevador.ResetEncoders;
import frc.robot.commands.Chasis.AlignToAprilTagCommand;
import frc.robot.commands.Chasis.ArcadeDriveCmd;
import frc.robot.commands.Chasis.Drive;
import frc.robot.commands.Chasis.RotarDerecha;
import frc.robot.commands.Chasis.RotarIzquierda;
import frc.robot.commands.Elevador.ElevatorCmd;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Elevator;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chasis chasis = new Chasis();
  private final Elevator elevator = new Elevator();
  private final Vision vision = new Vision();
  private final XboxController control_1 = new XboxController(1);
  private final XboxController control_2 = new XboxController(0);
  
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
  private static final double ELEVATOR_RAISED_THRESHOLD = 100; // Velocity in base mesurements
  private static final double SLOW_FACTOR = 0.5; // Slow factor meanwhile elevator is elevated
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Add CommandScheduler to SmartDashboard
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

  private void configureBindings() {
    // Controller 1 - Driver
      // Add vision alignment button - change to continuous following for testing
      new JoystickButton(control_1, 3)
          .whileTrue(new AlignToAprilTagCommand(vision, chasis));
  
      new JoystickButton(control_1, 1)
          .whileTrue(new ElevatorCmd(elevator, 0.75, downlimitswitch, uplimitswitch)); // Subir
  
      new JoystickButton(control_1, 2)
          .whileTrue(new ElevatorCmd(elevator, -0.75, downlimitswitch, uplimitswitch)); // Bajar
      
      new JoystickButton(control_1, 5).onTrue(new ResetEncoders(elevator));    
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