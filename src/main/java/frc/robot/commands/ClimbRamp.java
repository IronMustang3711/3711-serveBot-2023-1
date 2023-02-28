package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class ClimbRamp extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private int stage;
    private double pitch, peakPitch;
    private double startTime;


   
    public ClimbRamp(DrivetrainSubsystem drivetrainSubsystem){

    m_drivetrainSubsystem=drivetrainSubsystem;
  

    addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stage = 0;
        peakPitch = 0;
        m_drivetrainSubsystem.drive(new ChassisSpeeds(-1.3, 0, 0));
    }
  
    @Override
    public void execute() {
        pitch = m_drivetrainSubsystem.getPitch();
        // double rawPitch = m_drivetrainSubsystem.getPitch();
        // double filter = 0.5;  // simple exponential filter      **** not used.
        // pitch = (filter * rawPitch) + ((1 - filter) * pitch); 
        switch(stage){

            case 0:  // approach ramp
            if (pitch > 10)  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            {
                stage = 1;  // on ramp, slow down
                m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.7, 0, 0));
            }
            break;

            case 1: // climb ramp
            if (pitch < (peakPitch - 10))  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            {
                stage = 2;  // on platform, reverse a little bit.
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0.7, 0, 0));
                startTime = Timer.getFPGATimestamp();
            }
            if (peakPitch < pitch)  // save peak
                peakPitch = pitch;
            break;

            case 2:  // backup a little to balance platform
            if ((Timer.getFPGATimestamp() - startTime) > 1.0) // stop after timeout of .3 seconds <<<<<<<<<<<<<<<<<<
            {
                stage = 3; // time to quit.  Lock wheels
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0.1));
            }
            break;

        }
     
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        if (stage == 3) 
            return true;  // quit when we lock wheels
        else
            return false;
    }
}
