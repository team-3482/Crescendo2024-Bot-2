// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    /** Latest Limelight data. May contain faulty data unsuitable for odometry. */
    private LimelightData[] limelightDatas = new LimelightData[2];
    /** Last heartbeat of the front LL (updated every frame) */
    private long lastHeartbeatStheno = 0;
    /** Last heartbeat of the back LL (updated every frame) */
    private long lastHeartbeatEuryale = 0;

    // Lists used for tag filtering. Final to reduce wasted processing power.
    private final List<Integer> BLUE_SOURCE = Arrays.asList(1, 2, 3, 4);
    private final List<Integer> RED_SPEAKER = Arrays.asList(1, 2, 3, 4, 5);
    private final List<Integer> RED_AMP = Arrays.asList(5, 4, 3);
    private final List<Integer> BLUE_AMP = Arrays.asList(6, 7, 8);
    private final List<Integer> BLUE_SPEAKER = Arrays.asList(6, 7, 8, 9, 10);
    private final List<Integer> RED_SOURCE = Arrays.asList(7, 8, 9, 10);

    /** Creates a new ExampleSubsystem. */
    private VisionSubsystem() {
        super("VisionSubsystem");
    }

    /**
     * Gets the most trustworthy data from each Limelight. Also updates {@link VisionSubsystem#limelightDatas} and heartbeats.
     * @return LimelightData for each trustworthy Limelight data.
     * @apiNote Will theoretically stop updating data when the heartbeat has reset.
     * However, this happens at 2e9 frames, which would take 96 days at a consistent 240 fps .
     */
    private LimelightData[] getLimelightData() {
        double rotationDegrees = TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(LimelightConstants.FRONT_APRIL_TAG_LL, rotationDegrees, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(LimelightConstants.BACK_APRIL_TAG_LL, rotationDegrees, 0, 0, 0, 0, 0);
        
        LimelightHelpers.PoseEstimate frontLLDataMT2 = null;
        LimelightHelpers.PoseEstimate backLLDataMT2 = null;
        long heartbeatStheno = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.FRONT_APRIL_TAG_LL, "hb").getInteger(-1);
        long heartbeatEuryale = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.BACK_APRIL_TAG_LL, "hb").getInteger(-1);

        if (heartbeatStheno == -1 || this.lastHeartbeatStheno < heartbeatStheno) {
            frontLLDataMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.FRONT_APRIL_TAG_LL);
            LimelightHelpers.PoseEstimate frontLLDataMT = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.FRONT_APRIL_TAG_LL);
            this.limelightDatas[0] = new LimelightData(LimelightConstants.FRONT_APRIL_TAG_LL, frontLLDataMT, frontLLDataMT2);
            this.lastHeartbeatStheno = heartbeatStheno == -1 ? this.lastHeartbeatStheno : heartbeatStheno;
        }

        if (heartbeatEuryale == -1 || this.lastHeartbeatEuryale < heartbeatEuryale) {
            backLLDataMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.BACK_APRIL_TAG_LL);
            LimelightHelpers.PoseEstimate backLLDataMT = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.BACK_APRIL_TAG_LL);
            this.limelightDatas[1] = new LimelightData(LimelightConstants.BACK_APRIL_TAG_LL, backLLDataMT, backLLDataMT2);
            this.lastHeartbeatEuryale = heartbeatEuryale == -1 ? this.lastHeartbeatEuryale : heartbeatEuryale;
        }

        ChassisSpeeds robotChassisSpeeds = TunerConstants.DriveTrain.getCurrentRobotChassisSpeeds();
        double velocity = Math.sqrt(Math.pow(robotChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(robotChassisSpeeds.vyMetersPerSecond, 2));
        // If our angular velocity is greater than 270 deg/s, translational velocity is over 2 m/s,
        // the data is outdated (no new heartbeat), or there is no data, ignore vision updates.
        if (Math.abs(Units.radiansToDegrees(robotChassisSpeeds.omegaRadiansPerSecond)) > 270
            || Math.abs(velocity) > 2 // m/s
            || (this.lastHeartbeatEuryale != heartbeatEuryale && this.lastHeartbeatStheno != heartbeatStheno)
            || ((frontLLDataMT2 == null || frontLLDataMT2.tagCount == 0)
                && (backLLDataMT2 == null || backLLDataMT2.tagCount == 0))
            || (frontLLDataMT2.avgTagDist > LimelightConstants.TRUST_TAG_DISTANCE
                && backLLDataMT2.avgTagDist > LimelightConstants.TRUST_TAG_DISTANCE)) {
            return new LimelightData[0];
        }

        // More tags = more accurate. Also eliminate the other if it happens to be invalid.
        if (this.lastHeartbeatEuryale != heartbeatEuryale
            || backLLDataMT2 == null
            || backLLDataMT2.avgTagDist > LimelightConstants.TRUST_TAG_DISTANCE
            || frontLLDataMT2.tagCount > backLLDataMT2.tagCount) {
                return new LimelightData[]{ this.limelightDatas[0] };
        }
        else if (this.lastHeartbeatStheno != heartbeatStheno
            || frontLLDataMT2 == null
            || frontLLDataMT2.avgTagDist > LimelightConstants.TRUST_TAG_DISTANCE
            || backLLDataMT2.tagCount > frontLLDataMT2.tagCount) {
                return new LimelightData[]{ this.limelightDatas[1] };
        }

        // If it's way closer, then trust only that one. 80% is a heuteristic
        if (frontLLDataMT2.avgTagDist < backLLDataMT2.avgTagDist * 0.8) {
            return new LimelightData[]{ this.limelightDatas[0] };
        }
        else if (backLLDataMT2.avgTagDist < frontLLDataMT2.avgTagDist * 0.8) {
            return new LimelightData[]{ this.limelightDatas[1] };
        }

        // Both have the same amount of tags, both are almost equadistant from tags
        return this.limelightDatas;
    }

    /**
     * A helper method used to optimize Limelight FPS.
     */
    private void optimizeLimelights() {
        byte index = 0; // Used only for setting the optimized flag, so that this can be a for-each loop.
        for (LimelightData limelightData : this.limelightDatas) {
            if (!limelightData.optimized) {
                this.limelightDatas[index++].optimized = true;
            }
            
            // Avoid unnecessary optimization for a LL with no tags
            // Reset any optimization that might have been done previously
            if (limelightData.MegaTag2 == null || limelightData.MegaTag2.tagCount == 0) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 1.5f);
                LimelightHelpers.SetFiducialIDFiltersOverride(limelightData.name, LimelightConstants.ALL_TAG_IDS);
                LimelightHelpers.setCropWindow(limelightData.name, -1, 1, -1, 1);
                continue;
            }

            // Tag filtering
            Set<Integer> nearbyTagsSet = new HashSet<Integer>();
            for (LimelightHelpers.RawFiducial fiducial : limelightData.MegaTag2.rawFiducials) {
                switch (fiducial.id) {
                    case 1:
                    case 2:
                        nearbyTagsSet.addAll(this.BLUE_SOURCE);
                    case 3:
                    case 4:
                        nearbyTagsSet.addAll(this.RED_SPEAKER);
                    case 5:
                        nearbyTagsSet.addAll(this.RED_AMP);
                    case 6:
                        nearbyTagsSet.addAll(this.BLUE_AMP);
                    case 7:
                    case 8:
                        nearbyTagsSet.addAll(this.BLUE_SPEAKER);
                    case 9:
                    case 10:
                        nearbyTagsSet.addAll(this.RED_SOURCE);
                    default: // 11, 12, 13, 14, 15, and 16 have no relevant tags near them.
                        nearbyTagsSet.add(fiducial.id);
                }
            }
            int[] nearbyTagsArray = nearbyTagsSet.stream().mapToInt(i -> i).toArray();
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightData.name, nearbyTagsArray);

            // Downscaling closer to tags
            if (limelightData.MegaTag2.avgTagDist < 1.75) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 3);
            }
            else if (limelightData.MegaTag2.avgTagDist < 2.5) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 2);
            }
            else {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 1.5f);
            }
            
            // Smart cropping follows AprilTags
            // See : https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#basic-targeting-data
            if (limelightData.MegaTag2.rawFiducials.length == 0) {
                LimelightHelpers.setCropWindow(limelightData.name, -1, 1, -1, 1);
            }
            else {
                LimelightHelpers.RawFiducial txncBig = null;
                LimelightHelpers.RawFiducial txncSmall = null;
                LimelightHelpers.RawFiducial tyncBig = null;
                LimelightHelpers.RawFiducial tyncSmall = null;
                double targetSize = 0;
                
                // Finds the txnc and tync that are furthest from the crosshair
                // (for largest bounding box that will include all targets on screen)
                for (LimelightHelpers.RawFiducial fiducial: limelightData.MegaTag2.rawFiducials) {
                    targetSize = Math.sqrt(fiducial.ta * LimelightConstants.FOV_AREA) / 2;
                    
                    if (txncBig == null || fiducial.txnc + targetSize > txncBig.txnc) {
                        txncBig = fiducial;
                    }
                    if (txncSmall == null || fiducial.txnc - targetSize < txncSmall.txnc) {
                        txncSmall = fiducial;
                    }
                    
                    if (tyncBig == null || fiducial.tync + targetSize > tyncBig.tync) {
                        tyncBig = fiducial;
                    }
                    if (tyncSmall == null || fiducial.tync - targetSize < tyncSmall.tync) {
                        tyncSmall = fiducial;
                    }
                }
                
                // The formulas used below create the bounding boxes around targets and work in this way :
                //
                // The position of the target (x or y) that was found to be the furthest from the principal pixel for that direction
                // (largest/smallest x and largest/smallest y)
                //     MINUS for the smallest positions (left/bottom of the box) or PLUS for the largest positions (right/top of the box)
                //         The length of the side of the targets — This is found in the following way :
                //           We know the FOV area (LimelightConstants.FOV_AREA) -> We know percentage of screen target occupies (ta) ->
                //           Targets are roughly squares at most angles so sqrt(target area in pixels) = side lengths
                //         Which is MULTIPLIED by a function that scales with distance (further away needs larger box due
                //         to bot movements having more impact on target position from the camera's POV) in the following way :
                //           1.75 (heuteristic, this determines general box size) * ln(distance to target + 1)
                //           The +1 is added to the natural log to avoid a negative value for distances of less than 1 meter,
                //           even if those are very rare. Natural log is probably not the best function for this, but it works well enough.
                //
                // Together this comes out as (be careful to follow order of operations) :
                // Target Position +/- Target Length * (1.75 * ln(Distance + 1))
                //
                // In the end this is DIVIDED by HALF of the rough width or height of the FOV,
                // because Limelights expect cropping to be [-1.0, 1.0].

                double xSmall = (txncSmall.txnc - Math.sqrt(txncSmall.ta * LimelightConstants.FOV_AREA) * (1.75 * Math.log(txncSmall.distToCamera + 1)))
                    / (LimelightConstants.FOV_X / 2);
                double xBig = (txncBig.txnc + Math.sqrt(txncBig.ta * LimelightConstants.FOV_AREA) * (1.75 * Math.log(txncBig.distToCamera + 1)))
                    / (LimelightConstants.FOV_X / 2);
                
                LimelightHelpers.setCropWindow(
                    limelightData.name,
                    getNearbyTagDirection(txncSmall.id) < 0 ? xSmall - 1.5 * Math.abs(xBig - xSmall) : xSmall,
                    getNearbyTagDirection(txncBig.id) > 0 ? xBig + 1.5 * Math.abs(xBig - xSmall) : xBig,
                    (tyncSmall.tync - Math.sqrt(tyncSmall.ta * LimelightConstants.FOV_AREA) * (2 * Math.log(tyncBig.distToCamera + 1)))
                        / (LimelightConstants.FOV_Y / 2),
                    (tyncBig.tync + Math.sqrt(tyncBig.ta * LimelightConstants.FOV_AREA) * (2 * Math.log(tyncSmall.distToCamera + 1)))
                        / (LimelightConstants.FOV_Y / 2)
                );
            }
        }
    }
    
    /**
     * This is a helper for {@link VisionSubsystem#optimizeLimelights(LimelightData[])} smart cropping.
     * @param id - The target ID to consider.
     * @return Whether to expect another tag on the left, right, or neither.
     * @apiNote Left : -1 ; Right : +1 ; Neither : 0.
     */
    private int getNearbyTagDirection(int id) {
        switch (id) {
            case 1:
            case 3:
            case 7:
            case 9:
                return -1;
            case 2:
            case 4:
            case 8:
            case 10:
                return 1;
            default:
                return 0;
        }
    };

    /**
     * Gets the most trustworthy data from each Limelight and returns a {@link Pose2d} object.
     * @return The most accurate {@link Pose2d}, or {@code null} if there is none.
     */
    public Pose2d getEstimatedPose() {
        LimelightData[] limelightDatas = getLimelightData();

        if (limelightDatas.length == 0) {
            System.err.println("getEstimatedPose() | NO LIMELIGHT DATA, DEFAULTING TO EMTPY POSE2D");
            return new Pose2d();
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
        // TODO BOT : Timing
        long startTime = System.currentTimeMillis();

        LimelightData[] limelightDatas = getLimelightData();

        System.out.printf("Get data : %d ms%n", System.currentTimeMillis() - startTime);

        for (LimelightData data : limelightDatas) {
            if (data.canTrustRotation()) {
                // Only trust rotational data
                TunerConstants.DriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(9999999, 9999999, 0.5));
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
        
        System.out.printf("Update position : %d ms%n", System.currentTimeMillis() - startTime);

        long optiStartTime = System.currentTimeMillis();
        optimizeLimelights();
        long finalTime = System.currentTimeMillis();
        System.out.printf("Optimize LLs : %d ms%n", finalTime - optiStartTime);
        System.out.printf("Final : %d ms%n", finalTime - startTime);
    }
}
