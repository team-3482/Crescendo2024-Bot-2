// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.limelights.DetectionSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;

/**
 * A command that wraps a PathPlanner command that
 * paths to the nearest visible note and turns to face it.
 * Accounts for increased accuraccy with decreased distance.
 */
public class DriveToNoteCommand extends Command {
    private final PathConstraints constraints = new PathConstraints(
        4.3, 2,
        Units.degreesToRadians(720), Units.degreesToRadians(540)
    );
    
    private Command pathingCommand;
    private Timer timer;
    private Pose2d currentNotePose;
    private boolean finished;

    /** Creates a new DriveToNoteCommand. */
    public DriveToNoteCommand() {
        setName("DriveToNoteCommand");
        this.timer = new Timer();
        // No requirements, because PathPlanner's Command requires the DriveTrain
        // This wrapper is only useful for creating the PathPlanner Path
        addRequirements();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.finished = false;
        this.timer.restart();
        
        Pose2d[] notePoses = DetectionSubsystem.getInstance().getRecentNotePoses();
        if (notePoses.length == 0) {
            this.finished = true;
            return;
        }

        this.currentNotePose = notePoses[0];
        
        this.pathingCommand = generatePath(this.currentNotePose);
        CommandScheduler.getInstance().schedule(this.pathingCommand);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!this.pathingCommand.isScheduled() || this.finished) return;
        
        // Run this check every 5 robot cycles / 3 Detection cycles
        if (this.timer.hasElapsed(0.1)) {
            this.timer.restart();
        }
        else {
            return;
        }

        Pose2d[] notePoses = DetectionSubsystem.getInstance().getRecentNotePoses();
        // Note is stale by over 1.5 s so it has been discarded.
        // Go by last known note position. 
        if (notePoses.length == 0) {
            return;
        }
        Pose2d newNotePose = notePoses[0];

        // If within 1 meter, the path should be nearly perfect.
        if (CommandSwerveDrivetrain.getInstance().getState().Pose.getTranslation()
            .getDistance(newNotePose.getTranslation()) <= 1
        ) {
            return;
        }
        // If deviated over 15 cm from the original Note, repath
        else if (this.currentNotePose.getTranslation().getDistance(newNotePose.getTranslation()) >= 0.15) {
            this.currentNotePose = newNotePose;
            
            CommandScheduler.getInstance().cancel(this.pathingCommand);
            this.pathingCommand = generatePath(this.currentNotePose);
            CommandScheduler.getInstance().schedule(this.pathingCommand);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (this.pathingCommand != null && this.pathingCommand.isScheduled()) {
            CommandScheduler.getInstance().cancel(this.pathingCommand);
        }
        this.timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.finished || !this.pathingCommand.isScheduled();
    }

    /**
     * Helper that generates a PathPlanner Command to path to the note's position.
     * @param notePose - The position to path to.
     * @return The PathPlanner Command.
     */
    private Command generatePath(Pose2d notePose) {
        Translation2d botTranslation = CommandSwerveDrivetrain.getInstance().getState().Pose.getTranslation();

        // Takes into account angles in quadrants II and III
        Rotation2d faceNoteRot = new Rotation2d(Math.atan2(
            notePose.getY() - botTranslation.getY(),
            notePose.getX() - botTranslation.getX()
        ));

        Pose2d botPose = new Pose2d(botTranslation, faceNoteRot);
        notePose = new Pose2d(notePose.getTranslation(), faceNoteRot);

        GoalEndState goalEndState = new GoalEndState(
            0, // End with enough speed to pick up the note
            faceNoteRot,
            true
        );

        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(botPose, notePose),
            this.constraints,
            goalEndState
        );
        path.preventFlipping = true; // Field-relative Note position won't change based on alliance
        
        return AutoBuilder.followPath(path);
    }
}
