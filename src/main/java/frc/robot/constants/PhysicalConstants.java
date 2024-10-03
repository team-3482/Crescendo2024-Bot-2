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

    /** Constants for the IntakeSubsystem. */
    public static final class IntakeConstants {
        public static final int LEFT_INTAKE_MOTOR_ID = 20;
        public static final int RIGHT_INTAKE_MOTOR_ID = 21;
        public static final int BEAM_BREAK_CHANNEL = 0;

        public static final double INTAKE_SPEED = 0.6;
    }

    /** Constants for the ShooterSubsystem. */
    public static final class ShooterConstants {
        public static final int TOP_SHOOTER_MOTOR_ID = 31;
        public static final int BOTTOM_SHOOTER_MOTOR_ID = 30;

        /** This is the gear ratio from the sensor to the rollers. */
        public static final double ROTOR_TO_MECHANISM_RATIO = (double) 25 / 9; // 50:18

        // TODO SHOOTER (2.?) : Reasonable Velocity Tolerance
        /** Tolerance for Commands using MotionMagic in rot/s. */
        public static final double VELOCITY_TOLERANCE = 0;

        // TODO SHOOTER (2.2) : Tune MotionMagic
        /** Gains used for MotionMagic slot 0. */
        public static final class ShooterSlot0Gains {
            public static final double kG = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        // TODO SHOOTER (2.1) : FIND MAX ACCEL / JERK
        /* More constants used with MotionMagic. */
        /** Max mechanism rotations per second. */
        public static final double CRUISE_SPEED = 0; // 0 = no max speed
        /** Max mechanism rotations per second^2 */
        public static final double ACCELERATION = 0;
        /** Max mechanism rotations per second^3 */
        public static final double JERK = 0;
    }

    /** Constants for the PivotSubsystem. */
    public static final class PivotConstants {
        public static final int LEFT_PIVOT_MOTOR_ID = 40;
        public static final int RIGHT_PIVOT_MOTOR_ID = 41;

        /** Upper soft stop angle in degrees. */
        public static final double UPPER_ANGLE_LIMIT = 90;
        /** Lower soft stop angle in degrees. */
        public static final double LOWER_ANGLE_LIMIT = 2.796678; // Hard stop
        
        /** This is the gear ratio from the sensor to the pivot. */
        public static final double ROTOR_TO_MECHANISM_RATIO = 100; // 5:1 -> 2:1 -> 10 : 1 = 100:1
        
        /** Tolerance for Commands using MotionMagic in degrees. */
        public static final double POSITION_TOLERANCE = 0.5;

        /** Gains used for MotionMagic slot 0. */
        public static final class PivotSlot0Gains {
            public static final double kG = 0.24;
            public static final double kS = 0.1;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 512;
            public static final double kI = 0;
            public static final double kD = 8;
        }

        /* More constants used with MotionMagic. */
        /** Max mechanism rotations per second. */
        public static final double CRUISE_SPEED = 0.5;
        /** Max mechanism rotations per second^2 (any higher causes the bot to move) */
        public static final double ACCELERATION = 1.25;
    }
}
