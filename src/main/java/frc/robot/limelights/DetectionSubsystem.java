// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelights;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.LimelightConstants.DetectionConstants;
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

    private ArrayList<DetectionData> recentDetectionDatas = new ArrayList<DetectionData>();

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Note Pose");
    private final DoubleArrayPublisher note1 = table.getDoubleArrayTopic("note1").publish();
    private final DoubleArrayPublisher note2 = table.getDoubleArrayTopic("note2").publish();
    private final DoubleArrayPublisher note3 = table.getDoubleArrayTopic("note3").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("DetectionSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "TOP"))
        .withSize(2, 3);
    private final GenericEntry widthDistanceEntry = shuffleboardLayout
        .add("Closest Width-D (m)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", DetectionConstants.MAX_NOTE_DISTANCE, "Num tick marks", 2))
        .withSize(2, 1)
        .withPosition(0, 0)
        .getEntry();
    private final GenericEntry pitchDistanceEntry = shuffleboardLayout
        .add("Closest Pitch-D (m)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", DetectionConstants.MAX_NOTE_DISTANCE, "Num tick marks", 2))
        .withSize(2, 1)
        .withPosition(0, 1)
        .getEntry();

    /** Creates a new DetectionSubsystem. */
    private DetectionSubsystem() {
        super("DetectionSubsystem");

        // Shuffleboard camera feed.
        if (LimelightConstants.PUBLISH_CAMERA_FEEDS) {
            HttpCamera frontLLCamera = new HttpCamera(
                LimelightConstants.NOTE_DETECTION_LL,
                "http://" + LimelightConstants.NOTE_DETECTION_LL + ".local:5800/stream.mjpg"
            );

            Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
                .add(LimelightConstants.NOTE_DETECTION_LL, frontLLCamera)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));
        }
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        DetectionData[] detectionDatasArray = getDetectionDatas();
        if (detectionDatasArray.length == 0) return;

        ArrayList<DetectionData> detectionDatasList = new ArrayList<DetectionData>();

        for (DetectionData data : detectionDatasArray) {
            if (data.canTrustData) {
                detectionDatasList.add(data);
            }
        }

        if (detectionDatasList.size() != 0) {
            this.recentDetectionDatas = detectionDatasList;

            if (DetectionConstants.PUBLISH_NOTE_POSES) {
                Pose2d[] recentNotePoses = getRecentNotePoses();
                if (recentNotePoses.length >= 3) {
                    this.note3.set(pose2dToDoubleArray(recentNotePoses[2]));
                }
                if (recentNotePoses.length >= 2) {
                    this.note2.set(pose2dToDoubleArray(recentNotePoses[1]));
                }
                if (recentNotePoses.length >= 1) {
                    this.note1.set(pose2dToDoubleArray(recentNotePoses[0]));
                }
            }
        }

        this.fieldTypePub.set("Field2d");
    }
    
    /**
     * Helper that turns a Pose2d into a NetworkTables-readable double array.
     * @param pose - The pose to convert.
     * @return The double array representation of the pose.
     * @apiNote The rotation is in radians.
     */
    private double[] pose2dToDoubleArray(Pose2d pose) {
        return new double[]{
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }

    /**
     * Gets the recently saved note positions.
     * @return The recently calculated note positions.
     * @apiNote If there is no new data, it uses data up to 5 seconds old.
     * New data always replaces old data.
     */
    public Pose2d[] getRecentNotePoses() {
        ArrayList<Pose2d> notePosesList = new ArrayList<Pose2d>();
        this.widthDistanceEntry.setDouble(0);
        this.pitchDistanceEntry.setDouble(0);
        
        for (DetectionData data : this.recentDetectionDatas) {
            if (data.timestamp >= Timer.getFPGATimestamp() - 5) {
                double distance;
                double widthDistance = getDistanceFromWidth(data.width, data.tx);
                double pitchDistance = getDistanceFromPitch(data.ty);
                
                if (data.canTrustWidth) {
                    distance = widthDistance;
                }
                else if (data.canTrustPitch) {
                    distance = pitchDistance;
                }
                else {
                    this.widthDistanceEntry.setDouble(0);
                    this.pitchDistanceEntry.setDouble(0);
                    continue;
                }

                if (distance <= DetectionConstants.MAX_NOTE_DISTANCE) {
                    this.widthDistanceEntry.setDouble(widthDistance);
                    this.pitchDistanceEntry.setDouble(pitchDistance);
                    notePosesList.add(getNotePose(distance, data.tx));
                }
            }
        }
        
        return notePosesList.toArray(new Pose2d[notePosesList.size()]);
    }


    /**
     * Helper that sorts detection data and returns an array of detected Notes.
     * @return Detection data.
     * @see {@link LimelightHelpers#getBotPoseEstimate(String limelightName, String entryName)}
     * for getting the data and timestamps.
     */
    private DetectionData[] getDetectionDatas() {
        DoubleArrayEntry rawDetectionsEntry = LimelightHelpers.getLimelightDoubleArrayEntry(LimelightConstants.NOTE_DETECTION_LL, "rawdetections");
        double tl = LimelightHelpers.getLimelightNTDouble(LimelightConstants.NOTE_DETECTION_LL, "tl");
        double cl = LimelightHelpers.getLimelightNTDouble(LimelightConstants.NOTE_DETECTION_LL, "cl");

        TimestampedDoubleArray tsValue = rawDetectionsEntry.getAtomic();
        double[] rawDetections = tsValue.value;
        ArrayList<DetectionData> detectionDatas = new ArrayList<DetectionData>();

        double adjustedTimestamp = (tsValue.timestamp / 1000000.0) - ((tl + cl) / 1000.0);

        for (int i = 0; i < rawDetections.length; i += 12) {
            double tx = rawDetections[i + 1];
            double ty = rawDetections[i + 2];

            double[] xCorners = new double[]{
                rawDetections[i + 4], // Top left
                rawDetections[i + 6], // Top right
            };

            double[] yCorners = new double[]{
                rawDetections[i + 5], // Top left
                rawDetections[i + 11], // Bottom left
            };

            detectionDatas.add(new DetectionData(tx, ty, xCorners, yCorners, adjustedTimestamp));
        }

        return detectionDatas.toArray(new DetectionData[detectionDatas.size()]);
    }

    /**
     * Calculates the distance of a note using its width.
     * This is more trustworthy than the height, because the width varies more.
     * @param noteWidth - The width of the note in pixels.
     * @param tx - Raw tx from the camera in degrees.
     * @return The distance to the note in meters.
     * @see {@link DetectionConstants#PIXEL_TO_RAD}.
     * @apiNote This function is mostly heuristic and has about 7 cm error above 0.5 meters.
     */
    private double getDistanceFromWidth(double noteWidth, double tx) {
        double theta = noteWidth / DetectionConstants.PIXEL_TO_RAD;
        
        double distance = DetectionConstants.NOTE_DIAMETER / Math.tan(theta);
        distance += DetectionConstants.NOTE_DIAMETER / 2;
        
        if (distance >= 0.5) {
            distance += DetectionConstants.WIDTH_DIST_ERROR_CORRECTION.apply(distance);
        }
        
        distance = Math.abs(distance / Math.cos(Units.degreesToRadians(1.75 * tx)));
        
        return distance;
    }

    /**
     * Calculates the distance of a note using the vertical angle.
     * This is less accurate and should be used when the full width of the note is unavailable.
     * @param ty - Raw ty from the camera in degrees.
     * @return The distance to the note in meters.
     */
    private double getDistanceFromPitch(double ty) {
        double pitch = DetectionConstants.LIMELIGHT_POSITION.getRotation().getY();
        double theta = Math.PI / 2 - Math.abs(pitch + Units.degreesToRadians(ty));
        double height = DetectionConstants.LIMELIGHT_POSITION.getZ(); // - DetectionConstants.NOTE_Height
        
        double distance = height * Math.tan(theta);
        distance += DetectionConstants.PITCH_DIST_ERROR_CORRECTION.apply(distance);
        
        return distance;
    }

    /**
     * Calculates the Pose2d of a note in field space.
     * @param distance - The distance to the note from the camera.
     * @param tx - Raw tx from the camera in degrees.
     * @return The Pose of the note.
     * @apiNote The Rotation2d component of this Pose2d is the heading that
     * the bot needs to face this Note based on the Limelight's tx only.
     */
    private Pose2d getNotePose(double distance, double tx) {
        double yaw = Math.PI / 2 + DetectionConstants.LIMELIGHT_POSITION.getRotation().getZ();
        double theta = yaw + Units.degreesToRadians(tx);
        boolean over90Deg = false;
        
        if (theta > Math.PI / 2) {
            theta = Math.PI - theta;
            over90Deg = true;
        }

        Pose2d botPose = TunerConstants.DriveTrain.getState().Pose;
        
        Translation2d cameraToNote = new Translation2d(
            distance * Math.sin(theta),
            distance * Math.cos(theta)
        );

        Translation2d botToNote = new Translation2d(
            DetectionConstants.LIMELIGHT_POSITION.getX() + cameraToNote.getX(),
            -DetectionConstants.LIMELIGHT_POSITION.getY()
                + cameraToNote.getY() * (over90Deg ? -1 : 1)
                
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
