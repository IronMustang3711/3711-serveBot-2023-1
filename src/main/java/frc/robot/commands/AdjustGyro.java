package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class AdjustGyro extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    double m_adjustment;
   
    public AdjustGyro (DrivetrainSubsystem drivetrainSubsystem, double adjustment){

    m_drivetrainSubsystem=drivetrainSubsystem;
  
    m_adjustment = adjustment;

   // addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        m_drivetrainSubsystem.adjustGyroscopeHeading(m_adjustment);
    }

    @Override
    public void end(boolean interrupted) {
    }   // Returns true when the command should end.
    
    
    @Override
    public boolean isFinished() {
        return false;
    }

}
