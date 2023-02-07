// Arms Subsystem controls Dual arms motors, single elbow motor and clam20-jan-23 started with RobotBuilder Version: 5.0
//// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

 // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
 // REV smartMax code copied from Alternate Encoder example  %rod
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;  

/**
 *
 */
public class Clamp extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private static final MotorType kMotorType = MotorType.kBrushed; // %r1 .kBrushless;
    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int ClampID= 32; 
    private static final int ClampCPR = 3249 / 121 * 7 * 4;  // 751.8~ for PG27  ratio x 7ppr x 4 ;  
  
    private CANSparkMax clamp_motor;
    private RelativeEncoder clamp_encoder;
  
    /**
    *
    */
    public Clamp() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
         // initialize SPARK MAX with CAN ID
    clamp_motor = new CANSparkMax(ClampID, kMotorType);
    clamp_motor.restoreFactoryDefaults();
    clamp_encoder = clamp_motor.getAlternateEncoder(kAltEncType, ClampCPR);
    
    SmartDashboard.putNumber("Set Rotations", 0);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void position(double clampPose)
    {
        double position = clamp_encoder.getPosition();

        double error = clampPose - position;
        error *= 0.005;  // proportional constant
        double limit = 0.45;
        if (error > limit)
          error = limit;
          else if (error < -limit)
            error = -limit;
       clamp_motor.set(error);  
       SmartDashboard.putNumber("EncoderC", position);
       SmartDashboard.putNumber("DriveC", error);

    }
    public void drive(double clampSpeed )
    {
        clamp_motor.set(clampSpeed);
    }

       
}
    

