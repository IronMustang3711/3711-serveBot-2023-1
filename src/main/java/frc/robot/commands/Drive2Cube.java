// this is a copy of drive2cone.  Must setup photonvision for cube.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive2Cube extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    double m_xSpeed;
    PhotonCamera camera = new PhotonCamera("usb1"); // %rod
    double turnLimit = 0.5;
    double m_fwdLimit = 0.3;  // may be parameter later...........
    boolean close = false;

    public Drive2Cube(DrivetrainSubsystem drivetrainSubsystem, double xSpeed){

    m_drivetrainSubsystem=drivetrainSubsystem;
    m_xSpeed=xSpeed;  // not used
     addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        camera.setPipelineIndex(1);  // select cube targeting
        close = false;
    }
  
    @Override
    public void execute() {
        // camera.setPipelineIndex(0);
        var result = camera.getLatestResult(); // this only looks for cone for now?????
        double turnDrive = 0;

        if (result.hasTargets()) {
            PhotonTrackedTarget target;
            target = result.getBestTarget();
            double yaw = target.getYaw();
            SmartDashboard.putNumber("Target Yaw", yaw);

            turnDrive = -yaw / 50; // this is the proportional constant
            if (turnDrive > turnLimit)// limit the drive to +/- 0.5
                turnDrive = turnLimit;
            else if (turnDrive < -turnLimit)
                turnDrive = -turnLimit;

            if (target.getArea() > 10) // close if cube is 10% of view
                close = true;

            if (close)  // slow straight if close.  steer and fast if not close
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0.3, 0, 0));
            else
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0.6, 0, turnDrive));
        }      
        else
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
     
    }

    @Override
    public void end(boolean interrupted) {  // stop drive
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
