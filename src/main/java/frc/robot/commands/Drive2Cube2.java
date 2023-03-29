// copied from drive2cone2.  Added close clamp and backup 21-mar-2023

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

public class Drive2Cube2 extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final Clamp m_clamp;

  double startTime;

  PhotonCamera camera = new PhotonCamera("usb1"); // %rod
  double turnLimit = 0.5;
  double m_fwdLimit = 0.3; // may be parameter later...........
  int stage = 0;

  public Drive2Cube2(DrivetrainSubsystem drivetrainSubsystem, Clamp clampSubsystem) {

    m_drivetrainSubsystem = drivetrainSubsystem;
    m_clamp = clampSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setPipelineIndex(1); // select cybe targeting
    stage = 0;
  
  }

  @Override
  public void execute() {
    // camera.setPipelineIndex(0);
    switch (stage) {
      case 0:
        var result = camera.getLatestResult(); // do we see a cone?
        double turnDrive = 0;

        if (result.hasTargets()) {
          PhotonTrackedTarget target;
          target = result.getBestTarget();
          double yaw = target.getYaw();
          SmartDashboard.putNumber("Target Yaw", yaw);
          // since camera is off center and angled, yaw needs a 15 degree correction
          turnDrive = -(yaw - 15) / 50; // this is the proportional constant
          if (turnDrive > turnLimit)// limit the drive to +/- 0.5
            turnDrive = turnLimit;
          else if (turnDrive < -turnLimit) {
            turnDrive = -turnLimit;
          }

    
          if (target.getArea() < 2) { // close if cube is 10% of view
            // keep steering toward cone
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.9, 0, turnDrive));
          } else {
            stage = 1;  // ok we are close, slow down
            startTime = Timer.getFPGATimestamp(); // start timer
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.35, 0, 0));
          }
        } else // can not see cone
          m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0, 0));
        break;

      case 1: // getting close
        // slow and straight for .7 seconds
        if ((Timer.getFPGATimestamp() - startTime) < .9) {
          m_drivetrainSubsystem.drive(new ChassisSpeeds(0.35, 0, 0));
        } else { // hopefully ready to clamp. Stop drive
          m_clamp.drive(0.7); // clamp it
          stage = 2;
          m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0, 0));
          startTime = Timer.getFPGATimestamp(); // start timer
        }
        break;

      case 2:  // ready to clamp
        if ((Timer.getFPGATimestamp() - startTime) < .6) {
          m_clamp.drive(0.7); // clamp it
        } else { // got it clamped, backup
          m_clamp.drive(0.05); // hold clamp
          stage = 3;
          m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.4, 0, 0));
          startTime = Timer.getFPGATimestamp(); // start timer
        }
        break;

      case 3:  // clamped, now backup
        if ((Timer.getFPGATimestamp() - startTime) < 1.1) {
          m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.5, 0, 0));
        } else { // should be clear stop
          stage = 10;  // may want to do auto stow sometime.
          m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0, 0));
          startTime = Timer.getFPGATimestamp(); // start timer
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) { // stop drive
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
