// this is a copy DefaultDriveCommand.  drive with fixed robot angle

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FixedHeadingCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationAngleSupplier;

    public FixedHeadingCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationAngleSupplier) {  // target angle
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationAngleSupplier = rotationAngleSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double target = m_rotationAngleSupplier.getAsDouble();
        SmartDashboard.putNumber("Target Heading", target);

        double heading = m_drivetrainSubsystem.getGyroscopeHeading();
        SmartDashboard.putNumber("Heading", heading);

        // get error in -180 to 180 degree range.
        double error = (target - heading);
        // following magic converts error to +/- 180 degree range.
        error = Math.copySign(1,error) * ((Math.abs(error) + 180) % 360 - 180); 
        // or -(((180 - degree) % 360 + 360) % 360) + 180
        SmartDashboard.putNumber("Heading Error", error);

        double limit = 2;
        double rotateDrive = error * 1;  // proportional constant = 1????
        if (rotateDrive > limit)  // drive max 20 degrees per second.
          rotateDrive = limit;
          else if (rotateDrive < -limit)
          rotateDrive = -limit;

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        rotateDrive,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
