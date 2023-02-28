// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import java.util.List;

public class DrivetrainSubsystem extends SubsystemBase {

      
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // 
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5820.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>MK2
   * This is a measure of how fast the robot ca4_n rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );
  
//   Translation2d m_frontLeftLocation = new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
//   Translation2d m_frontRightLocation = new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0);
//   Translation2d m_backLeftLocation = new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
//   Translation2d m_backRightLocation = new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0);

// // Creating my kinematics object using the module locations
//   SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
//     m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
//   );

Pose2d m_pose = new Pose2d(5.0, 13.5, getGyroscopeRotation());
SwerveDriveOdometry m_odometry;
// Creating my odometry object from the kinematics object. Here,
// our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing forward.
// SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), m_pose);

  
    // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to ase until it wraps back over to zero.
  // 
  // 
//  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private  SwerveModule m_frontLeftModule;
  private  SwerveModule m_frontRightModule;
  private  SwerveModule m_backLeftModule;
  private  SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    

    // can call to create your swerve moduls.
    // The method you use depends on what motors you are using.
    //
    // Mk4SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk4SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk4SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk4SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    m_frontLeftModule = new MkSwerveModuleBuilder()
                    .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                    .withSize(2, 4)
                                    .withPosition(0, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
                                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)   
                    .build();



    m_frontRightModule = new MkSwerveModuleBuilder()
                    .withLayout(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(2, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                        .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
                        .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)   
                    .build();
    m_backLeftModule = new MkSwerveModuleBuilder()
                    .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
                                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                                
                    .build();
    m_backRightModule = new MkSwerveModuleBuilder()
                    .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
                                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                                
                    .build();

    var modules = List.of(m_frontLeftModule,m_frontRightModule,m_backLeftModule,m_backRightModule);

    int stall_limit_steer = 15;
    int free_limit_steer = 20;
    int stall_limit_drive = 25;
    int free_limit_drive = 37;
  
    for(var module : modules){
      ((CANSparkMax)module.getDriveMotor()).setSmartCurrentLimit(stall_limit_drive,free_limit_drive);
      ((CANSparkMax)module.getDriveMotor()).burnFlash();
      ((CANSparkMax)module.getSteerMotor()).setSmartCurrentLimit(stall_limit_steer, free_limit_steer);
      ((CANSparkMax)module.getSteerMotor()).burnFlash();
    }
                 
                     m_odometry = new SwerveDriveOdometry(
                        m_kinematics, getGyroscopeRotation(),
                        new SwerveModulePosition[] {
                          m_frontLeftModule.getPosition(),
                          m_frontRightModule.getPosition(),
                          m_backLeftModule.getPosition(),
                          m_backRightModule.getPosition()
                        }, m_pose);  
   }

  

 
  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope(double angleOffset) {
    // 
     m_pigeon.setFusedHeading(angleOffset);
  


    // 
//    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    // 
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    // 
//    if (m_navx.isMagnetometerCalibrated()) {
//      // We will only get valid fused headings if the magnetometer is calibrated
//      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
//    }
//
//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
//    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
    getPitch();
  }

  public double getPitch(){
    double roll = m_pigeon.getRoll();
    SmartDashboard.putNumber("Pitch", roll);
    return roll;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    
    SmartDashboard.putNumber("X", m_pose.getX());
    SmartDashboard.putNumber("Y", m_pose.getY());
    SmartDashboard.putNumber("Rot", m_pose.getRotation().getDegrees());

    m_pose = m_odometry.update(
        getGyroscopeRotation(), 
        new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
              }
    );



  }
}
