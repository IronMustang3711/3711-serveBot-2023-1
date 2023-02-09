// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5175; // 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6762; //  Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 10; //  Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5; //  Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3; //  Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 18; //  Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(306.2109375);// -Math.toRadians(117.68554687500001); //  Measure and set front left steer o
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; //  Set front right drive ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; //  Set front right steer ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 16; //  Set front right stcoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(247.8515625); //  Measure and set front right steer o
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8; //  Set back left drive mo
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; //  Set back left steer mo
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 15; //  Set back left steer r ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(138.69140339931275); //  Measure and set back left steer o
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6; //  Set back right drive mD
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1; //  Set back right steer mD0
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 17; //  Set back right steeder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(354.99023437500006); //  Measure and set back right steer offset
}
