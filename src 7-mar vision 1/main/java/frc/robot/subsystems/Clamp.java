// Arms Subsystem controls Dual arms motors, single elbow motor and clam20-jan-23 started with RobotBuilder Version: 5.0
//// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

 // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
 // REV smartMax code copied from Alternate Encoder example  %rod
import com.revrobotics.RelativeEncoder;
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
    private static final int ClampID= 32; 
  
    private CANSparkMax clamp_motor;
    private SparkMaxPIDController pidClamp;
    private RelativeEncoder clamp_encoder;
   
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    /**
    *
    */
    public Clamp() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
         // initialize SPARK MAX with CAN ID
   
    // Neo550 w/ 210:1      
    clamp_motor = new CANSparkMax(ClampID, MotorType.kBrushless);
    clamp_motor.restoreFactoryDefaults();
    clamp_motor.setSmartCurrentLimit( 30, 40);
    pidClamp = clamp_motor.getPIDController();
    clamp_encoder = clamp_motor.getEncoder();
    
    // set PID coefficients
    pidClamp.setP(0.00015); // proportional for Neo 550 arm
    pidClamp.setI(0.0000);
    pidClamp.setD(0);
    pidClamp.setIZone(0);
    pidClamp.setFF(0);
    pidClamp.setOutputRange(-1, 1);

    // set Motion Magic
    int smartMotionSlot = 0; // ????????????????????????????????
    pidClamp.setSmartMotionMaxVelocity(10000, smartMotionSlot);
    pidClamp.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    pidClamp.setSmartMotionMaxAccel(10000, smartMotionSlot);
    pidClamp.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot); // ?????????????

     // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Clamp Encoder", clamp_encoder.getPosition());
        SmartDashboard.putNumber("Clamp Current", clamp_motor.getOutputCurrent());
   
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void position(double clampPose)
    {
     // magic elbow control
     pidClamp.setReference(clampPose, CANSparkMax.ControlType.kSmartMotion);
 
  
    }
    public void drive(double clampSpeed )
    {
        clamp_motor.set(-clampSpeed);
    }

       
}
    
