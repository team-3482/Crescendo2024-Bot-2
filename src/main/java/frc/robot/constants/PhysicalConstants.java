// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants used throughout the code specifically related to subsystems or unchangeable aspects of the bot.
 * @implSpec BACK of the bot is 180 degrees / the battery, use that as a reference for directions.
 */
public final class PhysicalConstants {
    /**
     * Constants for physical attributes of the robot.
     */
    public static final class RobotConstants {
        /** Name of the CTRE CAN bus (configured on the CANivore). */
        public static final String CTRE_CAN_BUS = "ctre";
    }

    /** Constants for limelight-related data. */
    public static final class LimelightConstants {
        /** Spams "Bad LL 2D/3D Pose Data" when no data is coming from the NetworkTableInstance for a LL. */
        public static final boolean SPAM_BAD_DATA = true;
        /** The distance within which to use Limelight data in meters. This is measured from tag to camera.*/
        public static final int TRUST_TAG_DISTANCE = 4;

        /** Front left Limelight (April Tags). */
        public static final String FRONT_APRIL_TAG_LL = "limelight-stheno";
        /** Front right Limelight (Note detection). */
        public static final String NOTE_DETECTION_LL = "limelight-medusa"; // TODO : configure detection limelight
        /** Back Limelight (April Tags). */
        public static final String BACK_APRIL_TAG_LL = "limelight-euryale";

        /** All valid tag IDs (used for tag filtering) */
        public static final int[] ALL_TAG_IDS = new int[]{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
        
        // See https://docs.limelightvision.io/docs/docs-limelight/hardware-comparison.
        /** Horizontal FOV of LL3G in degrees (used for smart cropping) */
        public static final double FOV_X = 82;
        /** Vertical FOV of LL3G in degrees (used for smart cropping) */
        public static final double FOV_Y = 56.2;
        /** FOV area of the LL3g in degrees squared (used for smart cropping) */
        public static final double FOV_AREA = FOV_X * FOV_Y;
    }

    /** Constants for the intake subsystem. */
    public static final class IntakeConstants {
        public static final int LEFT_INTAKE_MOTOR_ID = 100;
        public static final int RIGHT_INTAKE_MOTOR_ID = 101;
        
        public static final double INTAKE_SPEED = 0.1;
    }

    /** Constants for the shooter subsystem. */
    public static final class ShooterConstants {
        public static final int TOP_SHOOTER_MOTOR_ID = 100;
        public static final int BOTTOM_SHOOTER_MOTOR_ID = 101;

        public static final double SHOOTER_SPEED = 0.3;
    }
}
