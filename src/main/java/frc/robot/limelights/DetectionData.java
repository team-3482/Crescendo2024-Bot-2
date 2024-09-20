package frc.robot.limelights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.LimelightConstants.DetectionConstants;
import frc.robot.swerve.TunerConstants;

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
    public final boolean canTrustData;

    /**
     * Creates a new DetectionData object.
     */
    public DetectionData(double tx, double ty, double[] xCorners, double timestamp) {
        this.tx = tx;
        this.ty = ty;
        this.width = ((xCorners[1] - xCorners[0]) + (xCorners[2] - xCorners[3])) / 2;
        this.timestamp = timestamp;

        // These are calculated when the measurement is created
        this.canTrustWidth = canTrustWidth(xCorners);
        this.canTrustData = canTrustData();
    }

    /**
     * Checks that the corners of the bounding box are not close to then edge of the screen.
     * @param xCorners - The corners to use. TL, TR, BR, BL.
     * @return If the full width is within the screen.
     */
    private boolean canTrustWidth(double[] xCorners) {
        // TODO : check if width is origin at crosshair or bottom left
        return xCorners[0] - 10 > 0
            && xCorners[3] - 10 > 0
            && xCorners[1] + 10 < DetectionConstants.SCREEN_WIDTH
            && xCorners[2] + 10 < DetectionConstants.SCREEN_WIDTH;
    }

    /**
     * Checks if the bot's rotational and translational velocities
     * are reasonable for trusting detection data.
     * @return Whether detection data can be trusted.
     * @apiNote Angular <= 160 deg/s ; Translational <= 2 m/s.
     */
    private boolean canTrustData() {
        ChassisSpeeds robotChassisSpeeds = TunerConstants.DriveTrain.getCurrentRobotChassisSpeeds();
        double velocity = Math.sqrt(Math.pow(robotChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(robotChassisSpeeds.vyMetersPerSecond, 2));
        return Units.radiansToDegrees(robotChassisSpeeds.omegaRadiansPerSecond) <= 160
            && velocity <= 2;
    }
}