package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class SetGyro extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    double m_angle;
   
    public SetGyro (DrivetrainSubsystem drivetrainSubsystem, double gyroAngle){

    m_drivetrainSubsystem=drivetrainSubsystem;
    m_angle = gyroAngle;

    addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrainSubsystem.zeroGyroscope(m_angle);
    }
  
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }   // Returns true when the command should end.
    
    @Override
    public boolean isFinished() {
        return true;  // finish immediatedly
    }

}
