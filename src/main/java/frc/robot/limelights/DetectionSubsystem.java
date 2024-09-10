// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelights;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        // TODO : DetectionData.java
        double[] widths = getWidths();
        double[] tys = getTYs();
        double[] txs = getTXs();

        for (int i = 0; i < widths.length || i < tys.length; i++) {
            System.out.printf(
                "width : %s in ; pitch : %s in%n",
                i < widths.length ? getNotePose(Units.metersToInches(getDistanceFromWidth(widths[i])), txs[i]).getTranslation().toString() : -1,
                i < tys.length ? getNotePose(Units.metersToInches(getDistanceFromPitch(tys[i])), txs[i]).getTranslation().toString() : -1
            );
        }
    }
    
    /**
     * Helper that calculates widths from corner points of all on-screen targets.
     * @return The width of each target in pixels.
     */
    private double[] getWidths() {
        double[] rawDetections = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.NOTE_DETECTION_LL, "rawdetections")
            .getDoubleArray(new double[0]);
        ArrayList<Double> widths = new ArrayList<Double>();
        
        for (int i = 4; i < rawDetections.length; i += 12) {
            // Finds the difference between top right and top left corners (horizontal)
            double top = rawDetections[i+2] - rawDetections[i];
            // Finds the difference between bottom right and bottom left corners (horizontal)
            double bottom = rawDetections[i+4] - rawDetections[i+6];
            
            widths.add((top + bottom) / 2);
        }

        return widths.stream().mapToDouble(i -> i).toArray();
    }

    /**
     * Helper that gets the ty for every on-screen target.
     * @return The ty of each target in degrees.
     */
    private double[] getTYs() {
        double[] rawDetections = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.NOTE_DETECTION_LL, "rawdetections")
            .getDoubleArray(new double[0]);
        ArrayList<Double> tys = new ArrayList<Double>();

        for (int i = 2; i < rawDetections.length; i += 12) {
            tys.add(rawDetections[i]);
        }

        return tys.stream().mapToDouble(i -> i).toArray();
    }

    /**
     * Helper that gets the tx for every on-screen target.
     * @return The tx of each target in degrees.
     */
    private double[] getTXs() {
        double[] rawDetections = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.NOTE_DETECTION_LL, "rawdetections")
            .getDoubleArray(new double[0]);
        ArrayList<Double> txs = new ArrayList<Double>();

        for (int i = 1; i < rawDetections.length; i += 12) {
            txs.add(rawDetections[i]);
        }

        return txs.stream().mapToDouble(i -> i).toArray();
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
        return distance + DetectionConstants.DISTANCE_TO_CENTER_OF_NOTE;
    }

    /**
     * Calculates the distance of a note using the vertical angle.
     * This is less accurate but should be used when the full width of the note is unavailable.
     * @param ty - Raw ty from the camera in degrees.
     * @return The distance to the note in meters.
     */
    private double getDistanceFromPitch(double ty) {
        double pitch = DetectionConstants.LIMELIGHT_POSITION.getRotation().getY();
        double theta = Math.abs(pitch + Units.degreesToRadians(ty));
        double limelightHeight = DetectionConstants.LIMELIGHT_POSITION.getZ();
        
        double distance = (limelightHeight - DetectionConstants.NOTE_HEIGHT) / Math.tan(theta);
        return distance + DetectionConstants.DISTANCE_TO_CENTER_OF_NOTE;
    }

    /**
     * Calculates the Pose of a note.
     * @param distance - The distance to the note
     * @param tx - Raw tx from the camera in degrees.
     * @return The Pose of the note.
     * @apiNote The rotation is 0 because notes are circular.
     */
    private Pose2d getNotePose(double distance, double tx) {
        double yaw = -DetectionConstants.LIMELIGHT_POSITION.getRotation().getZ();
        double theta = yaw + Units.degreesToRadians(tx);
        
        Pose2d botPose = TunerConstants.DriveTrain.getState().Pose;

        Translation2d cameraToNote = new Translation2d(
            distance * Math.cos(theta),
            distance * Math.sin(theta)
        );

        Translation2d botToNote = new Translation2d(
            cameraToNote.getX() + DetectionConstants.LIMELIGHT_POSITION.getX(),
            cameraToNote.getY() - DetectionConstants.LIMELIGHT_POSITION.getY()
        );

        double theta2 = Math.atan(botToNote.getY() / botToNote.getX()) + botPose.getRotation().getRadians();
        double dBotToNote = Math.sqrt(Math.pow(botToNote.getY(), 2) + Math.pow(botToNote.getX(), 2));

        Translation2d botToNoteFieldSpace = new Translation2d(
            dBotToNote * Math.cos(theta2),
            dBotToNote * Math.sin(theta2)
        );

        Pose2d notePose = new Pose2d(
            new Translation2d(
                botPose.getX() + botToNoteFieldSpace.getX(), 
                botPose.getY() + botToNoteFieldSpace.getY() 
            ),
            new Rotation2d()
        );

        return notePose;
    }
}
