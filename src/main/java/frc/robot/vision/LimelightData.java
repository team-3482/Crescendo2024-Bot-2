package frc.robot.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.TunerConstants;

/**
 * <p>A helper class used for storing MegaTag and MegaTag2 data from a Limelight
 * to avoid unnecessary calls to the NetworkTables API.
 * <p>MT rotation should not be combined with MT2 pose,
 * because their timestamps may differ.
 */
public class LimelightData {
    public final String name;
    public final LimelightHelpers.PoseEstimate MegaTag;
    public final LimelightHelpers.PoseEstimate MegaTag2;
    /** Flag set after optimization to avoid re-optimizing data twice in a row on low FPS. */
    public boolean optimized;

    /**
     * Creates a new LimelightData object.
     * @param name - The name of this Limelight.
     * @param MegaTag data.
     * @param MegaTag2 data.
     */
    public LimelightData(String name, LimelightHelpers.PoseEstimate MegaTag, LimelightHelpers.PoseEstimate MegaTag2) {
        this.name = name;
        this.MegaTag = MegaTag;
        this.MegaTag2 = MegaTag2;
        this.optimized = false;
    }

    /**
     * Checks if the average tag distance and bot's rotational and translational velocities
     * are reasonable for trusting rotation data.
     * @return Whether rotation data can be trusted.
     * @apiNote Dist <= 3 meters ; Angular <= 160 deg/s ; Translational <= 2
     */
    public boolean canTrustRotation() {
        ChassisSpeeds robotChassisSpeeds = TunerConstants.DriveTrain.getCurrentRobotChassisSpeeds();
        double velocity = Math.sqrt(Math.pow(robotChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(robotChassisSpeeds.vyMetersPerSecond, 2));
                                        // 3 Meters
        return this.MegaTag2.avgTagDist <= 3
            && Units.radiansToDegrees(robotChassisSpeeds.omegaRadiansPerSecond) <= 160
            && velocity <= 2;
    }
}