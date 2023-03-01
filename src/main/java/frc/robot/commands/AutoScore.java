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
public class AutoScore extends SequentialCommandGroup {
  /** Creates a new AutoSequence1. */ 

  public AutoScore(Arms armSystem, Clamp clampSystem, DrivetrainSubsystem m_drivetrainSubsystem) {

    addCommands(

        new SequentialCommandGroup(
            new SetGyro(m_drivetrainSubsystem, 180), // Robot starts aimed at drivers
            new Close(clampSystem).withTimeout(0.5), // clamp cube
            
            new ArmsLevel3(armSystem).withTimeout(2.5), // Position at top step
            new LowerClamp(armSystem).withTimeout(1.5), // lower a bit.

            new ParallelCommandGroup(
                new Open(clampSystem).withTimeout(0.5), // drop cube
                new ArmsLevel3(armSystem).withTimeout(0.5)) // raise to be clear

       //     new LoadZone(armSystem).withTimeout(0.5), // get up to clear stuff
       //     new StowArms(armSystem).withTimeout(2.0) // stow it.
        )
    );
  }
}
