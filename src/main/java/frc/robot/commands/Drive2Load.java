package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive2Load extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    double m_xSpeed;
    PhotonCamera camera = new PhotonCamera("usb1"); // %rod
    double slewLimit = 0.5;
    double m_fwdLimit = 0.3;  // may be parameter later...........


    public Drive2Load(DrivetrainSubsystem drivetrainSubsystem, double xSpeed){

    m_drivetrainSubsystem=drivetrainSubsystem;
    m_xSpeed=xSpeed;
     addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }
  
    @Override
    public void execute() {
        // camera.setPipelineIndex(0);
        var result = camera.getLatestResult(); // this only looks for cone for now?????
        double slewDrive = 0;

        if (result.hasTargets()) {
            PhotonTrackedTarget target;
            target = result.getBestTarget();
            double yaw = target.getYaw();
            SmartDashboard.putNumber("Target Yaw", yaw);
            // the yaw can vary from -25 to +25 degrees.
            // make slewDrive vary from -2.5 to 2.5
            slewDrive = -yaw / 50;
            if (slewDrive > slewLimit)// limit the drive to +/- 0.5
                slewDrive = slewLimit;
            else if (slewDrive < -slewLimit)
                slewDrive = -slewLimit;
            // if (Math.abs(slewDrive) < 0.1) // if drive is less than 0.1 do nothing
            // slewDrive = 0;

            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.3, 0, slewDrive));
        }
        else
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
     
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
