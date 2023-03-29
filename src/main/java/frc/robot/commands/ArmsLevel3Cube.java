// copy of ArmsLevel3.  This is sets elevation for cube delivery in Auto.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import frc.robot.RobotContainer; // %r1

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.Arms;

public class ArmsLevel3Cube extends CommandBase {

    private final Arms m_arms;
    private double elbowPosition;
    private double armPosition;
    private double elbowTarget = 67;
    private double armTarget = 15;

    public ArmsLevel3Cube(Arms subsystem) {

        m_arms = subsystem;
        addRequirements(m_arms);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arms.setSignLEDs(2); // green
        m_arms.setLEDRelays(false, false, false, true); // #4 is for cam LEDs
        m_arms.setPostLevel(3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        elbowPosition = m_arms.getElbowPosition();
        armPosition = m_arms.getArmPosition();
        if (elbowPosition < 20) {
            m_arms.position(0, elbowTarget); // 73
        } else {
            m_arms.position(armTarget, elbowTarget);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_arms.drive(0,0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if ((Math.abs(elbowPosition - elbowTarget) < 1) &&  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                (Math.abs(armPosition - armTarget) < 1))
            return true;
        else
            return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
