// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
     * Gets the most trustworthy data similar to the periodic loop for this subsystem.
     * @return Estimated Pose2d by the most accurate limelight (could be innacurate still)
     */
    public Pose2d getEstimatedPose() {
        Rotation2d botRotation = TunerConstants.DriveTrain.getState().Pose.getRotation();
        LimelightHelpers.SetRobotOrientation(LimelightConstants.FRONT_APRIL_TAG_LL, botRotation.getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(LimelightConstants.BACK_APRIL_TAG_LL, botRotation.getDegrees(), 0, 0, 0, 0, 0);
        
        LimelightHelpers.PoseEstimate frontLLData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.FRONT_APRIL_TAG_LL);
        LimelightHelpers.PoseEstimate backLLData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.BACK_APRIL_TAG_LL);

        // If there is no data, ignore vision updates
        if (frontLLData.tagCount == 0 && backLLData.tagCount == 0) {
            return null;
        }

        // More tags = more accurate
        if (frontLLData.tagCount > backLLData.tagCount) {
            return new Pose2d(frontLLData.pose.getTranslation(), botRotation);
        }
        else if (backLLData.tagCount > frontLLData.tagCount) {
            return new Pose2d(backLLData.pose.getTranslation(), botRotation);
        }

        // If it's way closer, then trust only that one. 80% is a heuteristic
        if (frontLLData.avgTagDist < backLLData.avgTagDist * 0.8) {
            return new Pose2d(backLLData.pose.getTranslation(), botRotation);
        }
        else if (backLLData.avgTagDist < frontLLData.avgTagDist * 0.8) {
            return new Pose2d(frontLLData.pose.getTranslation(), botRotation);
        }

        // Both have the same amount of tags, both are almost equadistant from tags
        return new Pose2d(
            frontLLData.pose.getTranslation().plus(backLLData.pose.getTranslation()).div(2),
            botRotation
        );
    }

    /**
     * A helper method for updating odometry using {@link LimelightHelpers#PosteEstimate} data
     * @param data - Used for updating odometry
     */
    private void updateOdometry(LimelightHelpers.PoseEstimate... data) {
        TunerConstants.DriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));

        for (LimelightHelpers.PoseEstimate d : data) {
            if (d.avgTagDist < 2) {
                TunerConstants.DriveTrain.seedFieldRelative(
                    new Pose2d(
                        TunerConstants.DriveTrain.getState().Pose.getTranslation(),
                        d.pose.getRotation()
                    )
                );
            }

            TunerConstants.DriveTrain.addVisionMeasurement(
                d.pose,
                d.timestampSeconds
            );
        }
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        double rotationDegrees = TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(LimelightConstants.FRONT_APRIL_TAG_LL, rotationDegrees, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(LimelightConstants.BACK_APRIL_TAG_LL, rotationDegrees, 0, 0, 0, 0, 0);
        
        LimelightHelpers.PoseEstimate frontLLData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.FRONT_APRIL_TAG_LL);
        LimelightHelpers.PoseEstimate backLLData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.BACK_APRIL_TAG_LL);

        double angularVelocityRadians = TunerConstants.DriveTrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond;
        // If our angular velocity is greater than 720 degrees per second or there is no data, ignore vision updates
        if (Math.abs(Units.radiansToDegrees(angularVelocityRadians)) > 720
            || (frontLLData.tagCount == 0 && backLLData.tagCount == 0)) {
            return;
        }

        // More tags = more accurate
        if (frontLLData.tagCount > backLLData.tagCount) {
            updateOdometry(frontLLData);
            return;
        }
        else if (backLLData.tagCount > frontLLData.tagCount) {
            updateOdometry(backLLData);
            return;
        }

        // If it's way closer, then trust only that one. 80% is a heuteristic
        if (frontLLData.avgTagDist < backLLData.avgTagDist * 0.8) {
            updateOdometry(frontLLData);
            return;
        }
        else if (backLLData.avgTagDist < frontLLData.avgTagDist * 0.8) {
            updateOdometry(backLLData);
            return;
        }

        // Both have the same amount of tags, both are almost equadistant from tags
        updateOdometry(frontLLData, backLLData);
    }
}
