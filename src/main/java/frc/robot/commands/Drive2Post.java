// copied from drive2cone2.  Look for Cone Post on grid

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Clamp;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive2Post extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final Clamp m_clamp;

  double startTime;

  PhotonCamera camera = new PhotonCamera("usb2"); // %rod
  double turnLimit = 0.5;
  double m_fwdLimit = 0.3; // may be parameter later...........
  int stage = 0;

  public Drive2Post(DrivetrainSubsystem drivetrainSubsystem, Clamp clampSubsystem) {

    m_drivetrainSubsystem = drivetrainSubsystem;
    m_clamp = clampSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setPipelineIndex(2); // select post targeting
    // turn on LEDs

    stage = 0;

  }

  @Override
  public void execute() {
    // camera.setPipelineIndex(0);
    var result = camera.getLatestResult(); // do we see a post?
    double turnDrive = 0;

    if (result.hasTargets()) {
      PhotonTrackedTarget target;
      target = result.getBestTarget();
      double yaw = target.getYaw();
      SmartDashboard.putNumber("Target Yaw", yaw);

      turnDrive = -yaw / 50; // this is the proportional constant
      if (turnDrive > turnLimit)// limit the drive to +/- 0.5
        turnDrive = turnLimit;
      else if (turnDrive < -turnLimit) {
        turnDrive = -turnLimit;
      }

      switch (stage) {
        case 0:
          if (target.getArea() < 1) { // close if post reflector is small 1.5% of view <<<<<<<<<<<<<<<<<<<<
            // keep steering toward post
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.4, turnDrive, 0));
          } else { // ok we are close, slow down
            stage = 1;
          }
          break;

        case 1: // getting close
          if (target.getArea() < 2.8) { // close, slow down, but keep steering <<<<<<<<<<<<<<<<<<
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.20, 0, turnDrive));
          } else { // on target, stoptur
            stage = 2;  // may want to open and backup if this works  stage = 2;
            startTime = Timer.getFPGATimestamp(); // start timer
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0, 0));
          }
          break;

        case 2: // Open clamp
          if ((Timer.getFPGATimestamp() - startTime) < .6) {
            m_clamp.drive(-0.7); // open clamp, drop it
          } else { // dropped, backup
            stage = 3;
            m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.4, 0, 0));
            startTime = Timer.getFPGATimestamp(); // start timer
            m_clamp.drive(0);
          }
          break;

        case 3: // now backup
          if ((Timer.getFPGATimestamp() - startTime) < .3) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.4, 0, 0));
            
          } else { // should be clear stop
            stage = 10; // may want to do auto stow sometime.
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0, 0));
            startTime = Timer.getFPGATimestamp(); // start timer
            
          }
          break;
      }
    }
  }

  @Override
  public void end(boolean interrupted) { // stop drive
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    
  }
}
