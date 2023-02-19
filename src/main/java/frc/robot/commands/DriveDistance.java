package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DriveDistance extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    double m_xSpeed;
   
    public DriveDistance(DrivetrainSubsystem drivetrainSubsystem, double xSpeed){

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
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(new ChassisSpeeds(m_xSpeed, 0.0, 0.0));
     
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
