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
    }
}
