// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Positions;
import frc.robot.constants.Constants.ShootingConstants;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.utilities.ShotVector;

/** Shoots a Note autonomously. */
public class AutoShootCommand extends Command {
    private Translation3d speakerTranslation3d;
    private ShotVector shotVector;
    private boolean allowMovement;
    
    private boolean endEarly;
    private Timer timer;

    final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle_withDeadband = new CommandSwerveDrivetrain.FieldCentricFacingAngle_PID_Workaround()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Creates a new AutoShootCommand.
     * @param allowMovement - Whether to require the driving subsystem, which prevents the driver from moving.
     */
    public AutoShootCommand(boolean allowMovement) {
        setName("AutoShootCommand");

        this.allowMovement = allowMovement;
        this.shotVector = new ShotVector();

        // Use addRequirements() here to declare subsystem dependencies.
        if (this.allowMovement) {
            addRequirements(
                IntakeSubsystem.getInstance(),
                PivotSubsystem.getInstance(),
                ShooterSubsystem.getInstance()
            );
        }
        else {
            addRequirements(
                IntakeSubsystem.getInstance(),
                PivotSubsystem.getInstance(),
                ShooterSubsystem.getInstance(),
                CommandSwerveDrivetrain.getInstance()
            );
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.endEarly = false;

        try {
            this.speakerTranslation3d = Positions.getSpeakerTarget();
        }
        catch (RuntimeException e) {
            System.err.println("Alliance is empty ; cannot target SPEAKER.");
            this.endEarly = true;
            return;
        }

        this.timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.endEarly) return;

        SwerveDriveState botState = CommandSwerveDrivetrain.getInstance().getState();
        double botHeading = botState.Pose.getRotation().getDegrees();
        ChassisSpeeds botSpeeds = botState.speeds;

        // Ignore rotation because it should be none for accurate shooting either way.
        ShotVector botVector = new ShotVector(botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond, 0);
        this.shotVector = calculateInitialShotVector().minus(botVector);

        if (this.allowMovement) {
            // TODO : Yaw while moving.
        }
        else {
            CommandSwerveDrivetrain.getInstance().setControl(fieldCentricFacingAngle_withDeadband
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(this.shotVector.getYaw()))
            );
        }

        // TODO Check that the angle can even physically make the shot
        
        PivotSubsystem.getInstance().motionMagicPosition(this.shotVector.getPitch());
        ShooterSubsystem.getInstance().motionMagicVelocity(this.shotVector.getNormRps());

        if (PivotSubsystem.getInstance().withinTolerance(this.shotVector.getPitch())
            && ShooterSubsystem.getInstance().withinTolerance(this.shotVector.getNormRps())
            && Math.min(
                Math.abs(this.shotVector.getYaw() - botHeading),
                360 - Math.abs(this.shotVector.getYaw() - botHeading)
            ) <= ShootingConstants.FACING_ANGLE_TOLERANCE
            // TODO ? Check tolerances using actual vector compared to goal vector ? (maybe how orbit does it)
        ) {
            IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_INTAKE_VELOCITY);
        }

        if (!IntakeSubsystem.getInstance().frontLaserHasNote()) {
            this.timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setSpeed(0);
        ShooterSubsystem.getInstance().setSpeed(0);
        PivotSubsystem.getInstance().motionMagicPosition(PivotConstants.ABOVE_LIMELIGHT_ANGLE);
        
        this.timer.stop();
    }

    private ShotVector calculateInitialShotVector() {
        SwerveDriveState botState = CommandSwerveDrivetrain.getInstance().getState();
        Pose2d botPose = botState.Pose;

        double distance = botPose.getTranslation().getDistance(this.speakerTranslation3d.toTranslation2d());
        double yaw = Units.radiansToDegrees(Math.atan2(
            this.speakerTranslation3d.getY() - botPose.getY(),
            this.speakerTranslation3d.getX() - botPose.getX()
        ) + Math.PI);
        double velocity = ShootingConstants.CALCULATE_SHOOTER_VELOCITY.apply(distance);
        double pitch = ShootingConstants.CALCULATE_SHOOTER_PITCH.apply(distance, velocity);

        return ShotVector.fromYawPitchVelocity(yaw, pitch, velocity);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.endEarly || this.timer.hasElapsed(0.25);
    }
}
