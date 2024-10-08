// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Positions;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.swerve.CommandSwerveDrivetrain;

/** Shoots a Note. */
public class PivotCommand extends Command {
    private double position;
    private boolean calculatePosition;
    private boolean end;

    private Translation3d speakerTranslation;
    private final double fallbackPosition;

    /**
     * Creates a new PivotCommand.
     * @param position - The position to pivot to. Used as a fallback if position cannot be calculated.
     * @param calculatePosition - Whether or not to calculate the position on-the-fly (aim for SPEAKER).
     * @param end - Whether to end the Command when at a that position, or to continue adjusting position until interrupted.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    private PivotCommand(double position, boolean calculatePosition, boolean end) {
        setName("PivotCommand");
        
        this.position = position;
        this.fallbackPosition = position;
        this.calculatePosition = calculatePosition;
        this.end = end;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(PivotSubsystem.getInstance());
    }

    /**
     * Creates a new PivotCommand.
     * @param position - The position to pivot to.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    private PivotCommand(double position) {
        this(position, false, true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.position = this.fallbackPosition;

        if (!this.calculatePosition) {
            PivotSubsystem.getInstance().motionMagicPosition(this.position);
            return;
        }

        try {
            this.speakerTranslation = Positions.getSpeakerTarget();
        }
        catch (RuntimeException e) {
            e.printStackTrace();
            this.speakerTranslation = null;
            PivotSubsystem.getInstance().motionMagicPosition(this.position);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!this.calculatePosition || this.speakerTranslation == null) return;

        Translation2d botTrans2d = CommandSwerveDrivetrain.getInstance().getState().Pose.getTranslation();

        double h = this.speakerTranslation.getZ() - PivotConstants.PIVOT_HEIGHT;
        double d = botTrans2d.getDistance(this.speakerTranslation.toTranslation2d());

        this.position = Units.radiansToDegrees(Math.atan(h / d));

        PivotSubsystem.getInstance().motionMagicPosition(this.position);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.end && PivotSubsystem.getInstance().withinTolerance(this.position);
    }
}
