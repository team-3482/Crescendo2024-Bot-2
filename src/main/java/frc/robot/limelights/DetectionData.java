package frc.robot.limelights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.LimelightConstants.DetectionConstants;
import frc.robot.swerve.CommandSwerveDrivetrain;

/**
 * <p>A helper class used for storing detection data for Notes.
 */
public class DetectionData {
    public final double tx;
    public final double ty;
    public final double width;
    public final double timestamp;

    /** If the bounding box is too close to the edge of the screen, it may not be the full note. */
    public final boolean canTrustWidth;
    /** If the bounding box is too close to the edge of the screen, it may not be the full note. */
    public final boolean canTrustPitch;
    public final boolean canTrustData;

    /**
     * Creates a new DetectionData object.
     * @apiNote If data cannot be trusted (see {@link DetectionData#canTrustData()}),
     * all booleans will be false and all doubles will be 0.
     */
    public DetectionData(double tx, double ty, double[] xCorners, double[] yCorners, double timestamp) {
        // These are calculated when the measurement is created
        this.canTrustWidth = canTrustWidth(xCorners);
        this.canTrustPitch = canTrustPitch(yCorners);
        this.canTrustData = canTrustData() && (this.canTrustWidth || this.canTrustPitch);
        
        if (!this.canTrustData) {
            this.tx = this.ty = this.width = this.timestamp = 0;
        }
        else {
            this.tx = tx;
            this.ty = ty;
            this.width = xCorners[1] - xCorners[0]; // Top right - Top left
            this.timestamp = timestamp;
        }
    }

    /**
     * Checks that the corners of the bounding box are not close to the edge of the screen.
     * @param xCorners - The corners to use. [ TL, TR ].
     * @return If the full width is within the screen.
     * @apiNote Within 35 px of the edges.
     */
    private boolean canTrustWidth(double[] xCorners) {
        return xCorners[0] - 35 > 0
            && xCorners[1] + 35 < DetectionConstants.SCREEN_WIDTH;
    }
    /**
     * Checks that the corners of the bounding box are not close to the edge of the screen.
     * @param yCorners - The corners to use. [ TL, BL ].
     * @return If the full height is within the screen.
     * @apiNote Within 15 px of the top edge.
     */
    private boolean canTrustPitch(double[] yCorners) {
        return yCorners[0] + 25 < DetectionConstants.SCREEN_HEIGHT;
            // && yCorners[1] - 35 > 0; // Trust pitch close-up
    }

    /**
     * Checks if the bot's rotational and translational velocities
     * are reasonable for trusting detection data.
     * @return Whether detection data can be trusted.
     * @apiNote Angular <= 160 deg/s ; Translational <= 2 m/s.
     */
    private boolean canTrustData() {
        ChassisSpeeds robotChassisSpeeds = CommandSwerveDrivetrain.getInstance().getCurrentRobotChassisSpeeds();
        double velocity = Math.sqrt(Math.pow(robotChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(robotChassisSpeeds.vyMetersPerSecond, 2));
        return Units.radiansToDegrees(robotChassisSpeeds.omegaRadiansPerSecond) <= 160
            && velocity <= 2;
    }
}