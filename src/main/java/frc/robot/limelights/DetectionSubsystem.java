// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelights;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.constants.PhysicalConstants.LimelightConstants.DetectionConstants;
import frc.robot.swerve.TunerConstants;

/** A class that manages note detection and associated calculations. */
public class DetectionSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile DetectionSubsystem instance;
    private static Object mutex = new Object();

    public static DetectionSubsystem getInstance() {
        DetectionSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new DetectionSubsystem();
                }
            }
        }
        return instance;
    }

    /** Creates a new DetectionSubsystem. */
    private DetectionSubsystem() {
        super("DetectionSubsystem");

        // Shuffleboard camera feed.
        HttpCamera frontLLCamera = new HttpCamera(
            LimelightConstants.NOTE_DETECTION_LL,
            "http://" + LimelightConstants.NOTE_DETECTION_LL + ".local:5800/stream.mjpg"
        );

        Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
            .add(LimelightConstants.NOTE_DETECTION_LL, frontLLCamera)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // TODO BOT : Test distance from width
        /** Sidelength of longest side of the fitted bounding box (pixels) */
        double tlong = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.NOTE_DETECTION_LL, "tlong")
            .getDouble(0);
        if (tlong != 0) {
            System.out.printf("tlong : %f px ; distance : %f m", tlong, getDistanceFromWidth(tlong));
        }

        // TODO BOT : Test distance from pitch
        double ty = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.NOTE_DETECTION_LL, "tync")
            .getDouble(0);
        if (ty != 0) {
            System.out.printf("ty : %f deg ; distance : %f m", ty, getDistanceFromPitch(ty));
        }
    }

    /**
     * Calculates the distance of a note using its width.
     * This is more trustworthy than the height, because the width varies more.
     * @param noteWidth - The width of the note in pixels.
     * @return The distance to the note in meters.
     * @see {@link DetectionConstants#PIXEL_TO_RAD}.
     */
    private double getDistanceFromWidth(double noteWidth) {
        double theta = noteWidth / DetectionConstants.PIXEL_TO_RAD;
        
        double distance = DetectionConstants.NOTE_DIAMETER / Math.tan(theta);
        return distance;
    }

    /**
     * Calculates the distance of a note using the vertical angle.
     * This is less accurate but should be used when the full width of the note is unavailable.
     * @param ty - Raw ty from the camera in degrees.
     * @return The distance to the note in meters.
     */
    private double getDistanceFromPitch(double ty) {
        double pitch = DetectionConstants.LIMELIGHT_POSITION.getRotation().getY();
        double theta = pitch + Units.degreesToRadians(ty);
        double limelightHeight = DetectionConstants.LIMELIGHT_POSITION.getZ();
        
        double distance = (limelightHeight - DetectionConstants.NOTE_HEIGHT) * Math.tan(theta);
        return distance;
    }

    /**
     * Calculates the Pose of a note.
     * @param distance - The distance to the note
     * @param tx - Raw tx from the camera in degrees.
     * @return The Pose of the note.
     * @apiNote The rotation is 0 because notes are circular.
     */
    private Pose2d getNotePose(double distance, double tx) {
        // TODO BOT : Shuffleboard step-by-step updates of each position to verify math

        double yaw = DetectionConstants.LIMELIGHT_POSITION.getRotation().getZ();
        double theta = yaw + Units.degreesToRadians(tx);
        
        Pose2d botPose = TunerConstants.DriveTrain.getState().Pose;
        Transform2d cameraTransform = new Transform2d(
            DetectionConstants.LIMELIGHT_POSITION.getTranslation().toTranslation2d(),
            DetectionConstants.LIMELIGHT_POSITION.getRotation().toRotation2d()
        );

        Pose2d cameraPose = botPose.transformBy(cameraTransform);

        double forward = Math.cos(theta) * distance;
        double sideways = Math.sin(theta) * distance;

        Transform2d noteTransform = new Transform2d(
            new Translation2d(forward, sideways),
            new Rotation2d()
        );

        Pose2d notePose = cameraPose.transformBy(noteTransform);

        return new Pose2d(
            notePose.getTranslation(),
            new Rotation2d()
        );
    }
}