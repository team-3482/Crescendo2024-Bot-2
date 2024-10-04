// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Constants for Limelight-related code. */
public final class LimelightConstants {
    /** Spams "Bad LL 2D/3D Pose Data" when no data is coming from the NetworkTableInstance for a LL. */
    public static final boolean SPAM_BAD_DATA = true;
    /** Enables publishing of the CameraFeeds to Shuffleboard on startup. */
    public static final boolean PUBLISH_CAMERA_FEEDS = true;
    
    /** Front left Limelight (April Tags). */
    public static final String FRONT_APRIL_TAG_LL = "limelight-stheno";
    /** Front right Limelight (Note detection). */
    public static final String NOTE_DETECTION_LL = "limelight-medusa";
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
        /** Whether or not the DetectionSubsystem will publish the top 3 most recent Notes. */
        public static final boolean PUBLISH_NOTE_POSES = true;
        /** Do not trust any note positions above this distance from the camera. In meters. */
        public static final double MAX_NOTE_DISTANCE = 3.5;

        /**
         * The real-life outer diameter of a note in meters.
         * @see Page 34 of the 2024 game manual :
         * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf
         */
        public static final double NOTE_DIAMETER = 0.36;
        /** The real-life height of a note in meters.
         * @see Page 34 of the 2024 game manual :
         * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf 
         */
        public static final double NOTE_HEIGHT = 0.05;

        /** The position of the Limelight relative to the center of the bot. */
        public static final Pose3d LIMELIGHT_POSITION = new Pose3d(
            new Translation3d(0.34, 0.25, 0.27),
            new Rotation3d(
                0,
                Units.degreesToRadians(-20),
                Units.degreesToRadians(-20)
            )
        );

        /** Heuristic conversion factor. Works with Limelight Resolution 1280x960 */
        public static final double PIXEL_TO_RAD = 900;

        /** The width of the screen in pixels, based on resolution. */
        public static final int SCREEN_WIDTH = 1280;

        /**
         * A formula that attempts to account for the detection error of far-away notes. 
         * This should only be applied for notes 0.5 m or more away.
         * @see https://www.desmos.com/calculator/zcrdcjewgg
         */
        public static Function<Double, Double> WIDTH_DIST_ERROR_CORRECTION = (Double d) -> {
            final double m = 0.058;
            final double b = m * Math.exp(0.5);
            return m * Math.exp(d) - b;
        };
    }
}