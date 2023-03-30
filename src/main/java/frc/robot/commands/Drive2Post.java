// copied from drive2cone2.  Look for Cone Post on grid

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Arms;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.DoubleSupplier;

import org.opencv.features2d.KAZE;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive2Post extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final Clamp m_clamp;
  private final Arms m_arms;

  double startTime;
  double level2DelayStartTime;

  PhotonCamera camera = new PhotonCamera("usb2"); // %rod
  double turnLimit = 0.5;
  double m_fwdLimit = 0.3; // may be parameter later...........
  int stage = 0;
  int postLevel = 0;
  double finalArea = 3;  

  public Drive2Post(DrivetrainSubsystem drivetrainSubsystem, Clamp clampSubsystem, Arms armsSubsystem) {

    m_drivetrainSubsystem = drivetrainSubsystem;
    m_clamp = clampSubsystem;
    m_arms = armsSubsystem;


    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {      
    m_arms.setLEDRelays(false, false, false, true); // #4 is for cam LEDs
    camera.setPipelineIndex(2); // select post targeting
    postLevel = m_arms.getPostLevel();
    if (postLevel == 2)
      finalArea = 1.8;  // if targetting mid level post
    else
      finalArea = 2.3;  // otherwise targetting level 3 post
    stage = 0;;
    SmartDashboard.putNumber("final area", finalArea);
  }

  @Override
  public void execute() {
    // camera.setPipelineIndex(0);
    var result = camera.getLatestResult(); // do we see a post?
    double turnDrive = 0;
    double area = -1; // no target if area < 0

    if (result.hasTargets()) {
      PhotonTrackedTarget target;
      target = result.getBestTarget();
      double yaw = target.getYaw();
      area = target.getArea();

      SmartDashboard.putNumber("Target Yaw", yaw);

      turnDrive = -yaw / 50; // this is the proportional constant
      if (turnDrive > turnLimit)// limit the drive to +/- 0.5
        turnDrive = turnLimit;
      else if (turnDrive < -turnLimit) {
        turnDrive = -turnLimit;
      }
    }

    switch (stage) {
      case 0:
        if (area > 0) { // do we have the target?
          if (area < 1) { // close if post reflector is small 1.% of view <<<<<<<<<<<<<<<<<<<<
            // keep steering toward post
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.4, turnDrive, 0));
          } else { // ok we are close, slow down
            stage = 1;
          }
        } else { // no target, stop
          m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, turnDrive, 0));
        }
        break;

      case 1: // getting close
        if (area > 0) { // do we have the target?
          if (area < finalArea) { // close, slow down, but keep steering <<<<<<<<<<<<<<<<<<
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.20, 0, turnDrive));
          } else { // on target, stop turn
            stage = 2; // may want to open and backup if this works stage = 2;
            startTime = Timer.getFPGATimestamp(); // start timer
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0, 0));
          }
        } else { // no target, stop
          m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, turnDrive, 0));
        }
        break;

      case 2: // Open clamp
        if ((Timer.getFPGATimestamp() - startTime) < .3) {
          m_clamp.drive(-0.6); // open clamp, drop it
        } else { // dropped, backup
          stage = 3;
          m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.6, 0, 0));
          startTime = Timer.getFPGATimestamp(); // start timer
          m_clamp.drive(0);
        }
        break;

      case 3: // now backup
        if ((Timer.getFPGATimestamp() - startTime) < .6) {
          m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.6, 0, 0));
          m_clamp.drive(0);
        } else { // should be clear stop
          stage = 10; // may want to do auto stow sometime.
          m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0, 0));
          startTime = Timer.getFPGATimestamp(); // start timer
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) { // stop drive
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    m_arms.setLEDRelays(false, false, false, false); // #4 is for cam LEDs
    if (stage >= 2) // be certain to shut of clamp drive if sequence aborted.
      m_clamp.drive(0);
  }
}
