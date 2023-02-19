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
public class AutoSequence1 extends SequentialCommandGroup {
  /** Creates a new AutoSequence1. */ 

  public AutoSequence1(Arms armSystem, Clamp clampSystem , DrivetrainSubsystem m_drivetrainSubsystem) {
   
    addCommands(

    new SequentialCommandGroup(
      new Close(clampSystem).withTimeout(0.5), // clamp cube
    //  new LoadZone(armSystem).withTimeout(2.0), // get up to clear stuff
      new ArmsLevel3(armSystem).withTimeout(2.0), // Position at top step
      new LowerClamp(armSystem ).withTimeout(1.0), // lower a bit.

      new ParallelCommandGroup(
        new Open(clampSystem).withTimeout(0.5), // drop cube
        new ArmsLevel3(armSystem).withTimeout(0.5)
      ) ,

      new LoadZone(armSystem).withTimeout(0.5), // get up to clear stuff
      new StowArms(armSystem).withTimeout(2.0), // stow it.
      
      new DriveDistance( m_drivetrainSubsystem, -.6 ).withTimeout(6))  // %r5 n test
      

      // new ParallelCommandGroup(
      //   new Open(clampSystem).withTimeout(0.6), // drive to tower in 6 seconds
      //   new LoadZone(armSystem).withTimeout(3.5)) // basically wait for 1 second
     );
  }
}
