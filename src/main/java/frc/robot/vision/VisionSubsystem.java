// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.LimelightConstants;
import frc.robot.swerve.TunerConstants;

public class VisionSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile VisionSubsystem instance;
    private static Object mutex = new Object();

    public static VisionSubsystem getInstance() {
        VisionSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new VisionSubsystem();
                }
            }
        }
        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    private VisionSubsystem() {
        super("VisionSubsystem");
    }

    /**
     * Gets the most trustworthy data from each Limelight.
     * @return LimelightData for each trustworthy Limelight data.
     */
    private LimelightData[] getLimelightData() {
        double rotationDegrees = TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(LimelightConstants.FRONT_APRIL_TAG_LL, rotationDegrees, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(LimelightConstants.BACK_APRIL_TAG_LL, rotationDegrees, 0, 0, 0, 0, 0);
        
        LimelightHelpers.PoseEstimate frontLLDataMT = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.FRONT_APRIL_TAG_LL);
        LimelightHelpers.PoseEstimate frontLLDataMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.FRONT_APRIL_TAG_LL);
        LimelightHelpers.PoseEstimate backLLDataMT = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.BACK_APRIL_TAG_LL);
        LimelightHelpers.PoseEstimate backLLDataMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.BACK_APRIL_TAG_LL);
        LimelightData[] limelightDatas = new LimelightData[0];

        double angularVelocityRadians = TunerConstants.DriveTrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond;
        // If our angular velocity is greater than 360 degrees per second or there is no data, ignore vision updates
        if (Math.abs(Units.radiansToDegrees(angularVelocityRadians)) > 360
            || ((frontLLDataMT2 == null || frontLLDataMT2.tagCount == 0)
                && (backLLDataMT2 == null || backLLDataMT2.tagCount == 0))
            || (frontLLDataMT2.avgTagDist > LimelightConstants.TRUST_TAG_DISTANCE
                && backLLDataMT2.avgTagDist > LimelightConstants.TRUST_TAG_DISTANCE)) {
            return limelightDatas;
        }

        // More tags = more accurate
        if (backLLDataMT2 == null
            || backLLDataMT2.avgTagDist > LimelightConstants.TRUST_TAG_DISTANCE
            || frontLLDataMT2.tagCount > backLLDataMT2.tagCount) {
            limelightDatas = new LimelightData[]{
                new LimelightData(LimelightConstants.FRONT_APRIL_TAG_LL, frontLLDataMT, frontLLDataMT2)
            };
            return optimizeLimelight(limelightDatas);
        }
        else if (frontLLDataMT2 == null
            || frontLLDataMT2.avgTagDist > LimelightConstants.TRUST_TAG_DISTANCE
            || backLLDataMT2.tagCount > frontLLDataMT2.tagCount) {
            limelightDatas = new LimelightData[]{
                new LimelightData(LimelightConstants.BACK_APRIL_TAG_LL, backLLDataMT, backLLDataMT2)
            };
            return optimizeLimelight(limelightDatas);
        }

        // If it's way closer, then trust only that one. 80% is a heuteristic
        if (frontLLDataMT2.avgTagDist < backLLDataMT2.avgTagDist * 0.8) {
            limelightDatas = new LimelightData[]{
                new LimelightData(LimelightConstants.FRONT_APRIL_TAG_LL, frontLLDataMT, frontLLDataMT2)
            };
            return optimizeLimelight(limelightDatas);
        }
        else if (backLLDataMT2.avgTagDist < frontLLDataMT2.avgTagDist * 0.8) {
            limelightDatas = new LimelightData[]{
                new LimelightData(LimelightConstants.BACK_APRIL_TAG_LL, backLLDataMT, backLLDataMT2)
            };
            return optimizeLimelight(limelightDatas);
        }

        // Both have the same amount of tags, both are almost equadistant from tags
        limelightDatas = new LimelightData[]{
            new LimelightData(LimelightConstants.FRONT_APRIL_TAG_LL, frontLLDataMT, frontLLDataMT2),
            new LimelightData(LimelightConstants.BACK_APRIL_TAG_LL, backLLDataMT, backLLDataMT2)
        };
        return optimizeLimelight(limelightDatas);
    }

    /**
     * A helper method used to optimize Limelight FPS.
     * @param limelightDatas - The data for the Limelights to optimize.
     * @return The input data for chaining.
     */
    private LimelightData[] optimizeLimelight(LimelightData[] limelightDatas) {
        // TODO : Smart cropping ?
        // TODO : Tag filtering ? (similar to smart cropping & pipeline switching)
        
        for (LimelightData limelightData : limelightDatas) {
            // TODO BOT : Test this
            // Pipeline switching when closer to tags
            if (limelightData.MegaTag2.avgTagDist < 5) {
                LimelightHelpers.setPipelineIndex(limelightData.name, LimelightConstants.PIPELINE_DOWNSCALE_5_METERS);
            }
            else if (limelightData.MegaTag2.avgTagDist < 10) {
                LimelightHelpers.setPipelineIndex(limelightData.name, LimelightConstants.PIPELINE_DOWNSCALE_10_METERS);
            }
            else {
                LimelightHelpers.setPipelineIndex(limelightData.name, LimelightConstants.PIPELINE_NORMAL);
            }
        }

        return limelightDatas;
    }

    /**
     * Gets the most trustworthy data from each Limelight and returns a {@link Pose2d} object.
     * @return The most accurate {@link Pose2d}, or {@code null} if there is none.
     */
    public Pose2d getEstimatedPose() {
        LimelightData[] limelightDatas = getLimelightData();

        if (limelightDatas.length == 0) {
            return null;
        }
        else if (limelightDatas.length == 1) {
            return new Pose2d(
                limelightDatas[0].MegaTag2.pose.getTranslation(),
                limelightDatas[0].canTrustRotation() ?
                    limelightDatas[0].MegaTag.pose.getRotation() : TunerConstants.DriveTrain.getState().Pose.getRotation()
            );
        }
        else {
            // Average them for best accuracy
            return new Pose2d(
                // (First translation + Second translation) / 2
                limelightDatas[0].MegaTag2.pose.getTranslation().plus(limelightDatas[1].MegaTag2.pose.getTranslation()).div(2),
                limelightDatas[0].canTrustRotation() ?
                    // First rotation / 2 + Second rotation / 2
                    /* 
                     * This is done to avoid problems due to Rotation2d being [0, 360) 
                     * Ex : 180+180=0 followed by 0/2=0 when it should be 180+180=360 and 360/2=180.
                     */
                    limelightDatas[0].MegaTag.pose.getRotation().div(2)
                        .plus(limelightDatas[1].MegaTag.pose.getRotation().div(2)) :
                    TunerConstants.DriveTrain.getState().Pose.getRotation()
            );
        }
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        LimelightData[] limelightDatas = getLimelightData();

        for (LimelightData data : limelightDatas) {
            if (data.canTrustRotation()) {
                // Only trust rotational data
                TunerConstants.DriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(9999999, 9999999, 0.7));
                TunerConstants.DriveTrain.addVisionMeasurement(
                    data.MegaTag.pose,
                    data.MegaTag.timestampSeconds
                );
            }

            // Only trust positional data
            TunerConstants.DriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            TunerConstants.DriveTrain.addVisionMeasurement(
                data.MegaTag2.pose,
                data.MegaTag2.timestampSeconds
            );
        }
    }
}

// TODO : time methods in this file and try to improve efficiency ?