package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

public class CrossRamp extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private int stage;
    private double pitch, peakPitch;
    private double startTime;


   
    public CrossRamp(DrivetrainSubsystem drivetrainSubsystem){

    m_drivetrainSubsystem=drivetrainSubsystem;
  

    addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stage = 0;
        peakPitch = 0;
        startTime = Timer.getFPGATimestamp();
        m_drivetrainSubsystem.drive(new ChassisSpeeds(-1.0, 0, 0));
    }
  
    @Override
    public void execute() {
        pitch = m_drivetrainSubsystem.getPitch();
        // double rawPitch = m_drivetrainSubsystem.getPitch();
        // double filter = 0.5;  // simple exponential filter      **** not used.
        // pitch = (filter * rawPitch) + ((1 - filter) * pitch); 
        // Note: robot backs up to get on ramp, so -x velocities are going backwards.
        switch(stage){

        case 0:  // approach ramp
            if ( (pitch > 10) ||
            ((Timer.getFPGATimestamp() - startTime) > 3.0)) 
            { // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                stage = 1; // on ramp, keep going
                m_drivetrainSubsystem.drive(new ChassisSpeeds(-.7, 0.0, 0.0)); // slow down
                startTime = Timer.getFPGATimestamp();
            }
            break;

        case 1: // climbing ramp, detect ramp drop
            if ((pitch < -5) || // decending ramp Slow a little
                    ((Timer.getFPGATimestamp() - startTime) > 3.0)) 
            {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.6, 0.0, 0.0)); // slow down
                stage = 2; // exiting platform.
                startTime = Timer.getFPGATimestamp();
            }
            break;

        case 2: // exiting ramp
            if ((pitch > -3) || // leveling out, slow down.
                    ((Timer.getFPGATimestamp() - startTime) > 3.0)) 
            {
                startTime = Timer.getFPGATimestamp();
                m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.4, 0.0, 0.0)); // slow down
                stage = 3;
            }
        
            break;

        case 3: // exit community

            if ((Timer.getFPGATimestamp() - startTime) > 0.5) // wait time to exit community <<<<<<<<<<<<<<<<<<
            { // reverse and go back up the platform
                stage = 4;
                m_drivetrainSubsystem.drive(new ChassisSpeeds(1.0, 0.0, 0.0)); // slow down
                startTime = Timer.getFPGATimestamp();
            }
            break;

        case 4: // go back up the ramp
            if ((pitch < -10) ||  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            ((Timer.getFPGATimestamp() - startTime) > 3.0)) 
            {             
                stage = 5; // on ramp, keep going
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0.6, 0.0, 0.0)); // slow down
                startTime = Timer.getFPGATimestamp();
            }
            break;

        case 5: // Platform is starting to level
            if ((Timer.getFPGATimestamp() - startTime) > 1.0) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0.3, 0, 0));

                if ((pitch > -10) || // pitch is dropping <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                  ((Timer.getFPGATimestamp() - startTime) > 3.0)) 
                {
                    stage = 6; // on platform, reverse a little bit.
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.4, 0, 0));
                    startTime = Timer.getFPGATimestamp();
                }
            }

        case 6: // backup a little to balance platform
            if ((Timer.getFPGATimestamp() - startTime) > 0.25) // stop after timeout of .3 seconds <<<<<<<<<<<<<<<<<<
            {
                stage = 7; // cock wheels and wait for platform to settle
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0.02, 0.0));
                startTime = Timer.getFPGATimestamp();
            }
            break;

        case 7:

            if ((Timer.getFPGATimestamp() - startTime) > 1) // wait time for settling platform <<<<<<<<<<<<<<<<<<
            {
                if (java.lang.Math.abs(pitch) > 2) // too much pitch, not level <<<<<<<<<<<<<<<<<<
                {
                    if (pitch > 0) // if pitch still + go backward
                        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.4, 0, 0));
                    else // otherwise go for forward.
                        m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.4, 0, 0));
                } else {
                    stage = 10; // time to quit. Lock wheels again.
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0.02, 0.0));
                    startTime = Timer.getFPGATimestamp();
                }
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
        if ((stage == 10) && ((Timer.getFPGATimestamp() - startTime) > 1.0)) // may not be needed
            return true;  // quit when we lock wheels
        else
            return false;
    }
}
