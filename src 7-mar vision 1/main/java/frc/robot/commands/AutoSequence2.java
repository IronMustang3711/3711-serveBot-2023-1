// 16-feb-2023 stole from Load Sequence  this is from bunnybot 2021.

// try to drop a cube onto the upper deck.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.DrivetrainSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSequence2 extends SequentialCommandGroup {
  /** Creates a new AutoSequence1. */ 

  public AutoSequence2(Arms armSystem, Clamp clampSystem , DrivetrainSubsystem m_drivetrainSubsystem) {
   
    addCommands(

    new SequentialCommandGroup(

      new AutoScore(armSystem, clampSystem, m_drivetrainSubsystem),
      new ClimbRamp( m_drivetrainSubsystem ).withTimeout(7.0), // give it 5 seconds <<<<<<<<<<<<<<     
      new StowArms(armSystem))
     );
  }
}
