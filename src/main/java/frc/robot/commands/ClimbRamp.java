package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class ClimbRamp extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private int stage;
    private double pitch, peakPitch;


   
    public ClimbRamp(DrivetrainSubsystem drivetrainSubsystem){

    m_drivetrainSubsystem=drivetrainSubsystem;
  

    addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stage = 0;
        peakPitch = 0;
    }
  
    @Override
    public void execute() {
        pitch = m_drivetrainSubsystem.getPitch();
        switch(stage){

            case 0:
            m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.6, 0, 0));
            if (pitch > 10)
                stage = 1;
            break;

            case 1:
            m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.3, 0, 0));
            if (pitch < (peakPitch - 10))
                stage = 2;
            if (peakPitch < pitch)

                peakPitch = pitch;
            break;

            case 2:
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0.1));
            break;

        }
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        // m_drivetrainSubsystem.drive(new ChassisSpeeds(m_xSpeed, 0.0, 0.0));
     
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        if ((stage == 2) && (pitch < 3))
            return true;
        else
            return false;
    }
}
