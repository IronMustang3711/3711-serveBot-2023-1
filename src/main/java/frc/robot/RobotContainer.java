// 3711.  ServeBot from 2023.  Removed everything but chassis.
// 12/5/23  new folder for chassis testing.
package frc.robot;

import frc.robot.commands.*;  
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // public final Arms m_arms = new Arms();  //%4
  // public final Clamp m_clamp = new Clamp();

  // private final XboxController m_controller_one = new XboxController(0);
  private final Joystick m_controller_one = new Joystick(0);
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller_one.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller_one.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller_one.getZ()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

  
    configureButtonBindings();

    // Configure default commands

    // Configure autonomous sendable chooser
   // m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   // Back button zeros the gyroscope
    // new Button(m_controller_one::getTrigger)
    //         // No requirements because we don't need to interrupt anything
    //         .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

// Create some buttons  //%r

// use trigger for fixed heading.
final JoystickButton zeroBtn = new JoystickButton(m_controller_one, 7);        
zeroBtn.onTrue(new SetGyro( m_drivetrainSubsystem, 0 ));  

// final JoystickButton driveBtn = new JoystickButton(m_controller_one, 7);        
// driveBtn.whileTrue(new DriveDistance( m_drivetrainSubsystem, -1 ));  


final JoystickButton ConeDriveBtn = new JoystickButton(m_controller_one, 9);    // old    
ConeDriveBtn.whileTrue(new Drive2Cone(m_drivetrainSubsystem, 0.5));

final JoystickButton Heading180Btn = new JoystickButton(m_controller_one, 2);
Heading180Btn.whileTrue(new FixedHeadingCommand(
    m_drivetrainSubsystem,
    () -> -modifyAxis(m_controller_one.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_controller_one.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> 180));

final JoystickButton Heading0Btn = new JoystickButton(m_controller_one, 1);
Heading0Btn.whileTrue(new FixedHeadingCommand(
    m_drivetrainSubsystem,
    () -> -modifyAxis(m_controller_one.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_controller_one.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> 0));

final JoystickButton CCWAdjustBtn = new JoystickButton(m_controller_one, 4);
CCWAdjustBtn.whileTrue(new AdjustGyro(m_drivetrainSubsystem, -0.3));

final JoystickButton CWAdjustBtn = new JoystickButton(m_controller_one, 5);
CWAdjustBtn.whileTrue(new AdjustGyro(m_drivetrainSubsystem, 0.3));

}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  //  public Joystick getJoystick() {
  //   return joystick;
  //  }

public Command getAutonomousCommand() {
  // The selected command will be run in autonomous from arms test %r
  return m_chooser.getSelected();
}

  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return new InstantCommand()
  // }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;

    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);
//This is where the fun begins
//General Kenobi, I assume you are here to...bring me to justice?
//You know me to well
    return value;
  }
}


