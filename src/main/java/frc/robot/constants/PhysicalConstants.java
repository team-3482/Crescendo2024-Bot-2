// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

    /** Constants for Limelight-related data. */
    public static final class LimelightConstants {
        /** Spams "Bad LL 2D/3D Pose Data" when no data is coming from the NetworkTableInstance for a LL. */
        public static final boolean SPAM_BAD_DATA = true;
        
        /** Front left Limelight (April Tags). */
        public static final String FRONT_APRIL_TAG_LL = "limelight-stheno";
        /** Front right Limelight (Note detection). */
        public static final String NOTE_DETECTION_LL = "limelight-medusa"; // TODO : configure detection limelight
        /** Back Limelight (April Tags). */
        public static final String BACK_APRIL_TAG_LL = "limelight-euryale";

        /** Constants used for AprilTag Limelights and vision algorithms. */
        public static final class VisionConstants {
            /** The distance within which to use Limelight data in meters. This is measured from tag to camera.*/
            public static final int TRUST_TAG_DISTANCE = 4;

            /** All valid tag IDs (used for tag filtering) */
            public static final int[] ALL_TAG_IDS = new int[]{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
            
            /** Crop window size when no tags are in view (used for smart cropping) */
            public static final double DEFAULT_CROP_SIZE = 0.85;
            
            // See https://docs.limelightvision.io/docs/docs-limelight/hardware-comparison.
            /** Horizontal FOV of LL3G in degrees (used for smart cropping) */
            public static final double FOV_X = 82;
            /** Vertical FOV of LL3G in degrees (used for smart cropping) */
            public static final double FOV_Y = 56.2;
            /** FOV area of the LL3g in degrees squared (used for smart cropping) */
            public static final double FOV_AREA = FOV_X * FOV_Y;
        }

        /** Constants used for the note detection Limelight and detection algorithms. */
        public static final class DetectionConstants {
            /**
             * The real-life outer diameter of a note in meters.
             * @see Page 34 of the 2024 game manual :
             * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf
             */
            public static double NOTE_DIAMETER = 0.36;
            /** The real-life height of a note in meters.
             * @see Page 34 of the 2024 game manual :
             * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf 
             */
            public static double NOTE_HEIGHT = 0.05;

            /** The position of the Limelight relative to the center of the bot. */
            public static Pose3d LIMELIGHT_POSITION = new Pose3d(
                new Translation3d(0.3, 0.23, 0.27), // TODO : Re-measure
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-20),
                    Units.degreesToRadians(-20)
                )
            );

            /** Heuristic conversion factor. Works with Limelight Resolution 1280x960 */
            public static double PIXEL_TO_RAD = 1150; // TODO : Re-measure
        }
    }
}
