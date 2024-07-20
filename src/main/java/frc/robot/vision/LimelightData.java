package frc.robot.vision;

import edu.wpi.first.math.util.Units;
import frc.robot.swerve.TunerConstants;

/**
 * A helper class used for storing MegaTag and MegaTag2 data from a Limelight
 * to avoid unnecessary calls to the NetworkTables API.
 */
public class LimelightData {
    public LimelightHelpers.PoseEstimate MegaTag;
    public LimelightHelpers.PoseEstimate MegaTag2;

    /**
     * Creates a new LimelightData object.
     * @param MegaTag data.
     * @param MegaTag2 data.
     */
    public LimelightData(LimelightHelpers.PoseEstimate MegaTag, LimelightHelpers.PoseEstimate MegaTag2) {
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