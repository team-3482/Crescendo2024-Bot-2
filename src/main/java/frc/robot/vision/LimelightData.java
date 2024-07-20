package frc.robot.vision;

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
    }

    /**
     * Checks if the average tag distance and bot's rotational velocity
     * are reasonable for trusting rotation data.
     * @return Whether rotation data can be trusted.
     */
    public boolean canTrustRotation() {
        double angularVelocityRadians = TunerConstants.DriveTrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond;
                                        // 3 Meters
        return this.MegaTag2.avgTagDist < 3 && Units.radiansToDegrees(angularVelocityRadians) <= 180;
    }
}