// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants used throughout the code specifically related to subsystems or unchangeable aspects of the bot.
 * @implNote BACK of bot is shooter/battery/180 deg, use that as reference for directions.
 */
public final class PhysicalConstants {
    /**
     * Constants of physical attributes of the robot.
     */
    public static final class RobotConstants {
        /** Name of the CTRE CAN bus. */
        public static final String CTRE_CAN_BUS = "ctre";
    }

    /** Constants for limelight-related data. */
    public static final class LimelightConstants {
        /** Spams "Bad LL 2D/3D Pose Data" when no data is coming from the NetworkTableInstance for LL. */
        public static final boolean SPAM_BAD_DATA = true;
        /** The distance within which to use Limelight data in meters. */
        public static final int TRUST_TAG_DISTANCE = 12;

        /** Front left Limelight (April Tags) */
        public static final String FRONT_APRIL_TAG_LL = "limelight-stheno";
        /** Front right Limelight (Note detection) */
        public static final String NOTE_DETECTION_LL = "limelight-medusa"; // TODO : configure detection limelight
        /** Back Limelight (April Tags) */
        public static final String BACK_APRIL_TAG_LL = "limelight-euryale";

        // All valid tag IDs
        public static final int[] ALL_TAG_IDS = new int[]{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
        // Field of view of LL3G, used for Smart Cropping
        public static final double FOV_X = 82;
        public static final double FOV_Y = 56.2;
        public static final double FOV_AREA = FOV_X * FOV_Y;
    }
}
