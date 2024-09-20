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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.limelights.DetectionSubsystem;
import frc.robot.swerve.TunerConstants;

/** A command that wraps a PathPlanner command that
 * paths to the nearest visible note and turns to face it. */
public class DriveToNoteCommand extends Command {
    private Command pathingCommand;
    private final PathConstraints constraints = new PathConstraints(
        4.45, 1,
        Units.degreesToRadians(270), Units.degreesToRadians(180)
    );
    private boolean finished;

    /** Creates a new DriveToNoteCommand. */
    public DriveToNoteCommand() {
        setName("DriveToNoteCommand");
        // No requirements, because PathPlanner's Command requires the DriveTrain
        // This wrapper is only useful for creating the PathPlanner Path
        addRequirements(); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.finished = false;
        
        Pose2d[] notePoses = DetectionSubsystem.getInstance().getRecentNotePoses();
        if (notePoses.length == 0) {
            this.finished = true;
            return;
        }

        Pose2d botPose = TunerConstants.DriveTrain.getState().Pose;
        Pose2d notePose = notePoses[0]; 

        GoalEndState goalEndState = new GoalEndState(
            0,
            // Takes into account angles in quadrants II and III
            new Rotation2d(Math.atan2(
                notePose.getY() - botPose.getY(),
                notePose.getX() - botPose.getX()
            )),
            true
        );

        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(botPose, notePose),
            this.constraints,
            goalEndState
        );
        path.preventFlipping = true;
        
        this.pathingCommand = AutoBuilder.followPath(path);
        CommandScheduler.getInstance().schedule(this.pathingCommand);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (this.pathingCommand.isScheduled()) {
            CommandScheduler.getInstance().cancel(this.pathingCommand);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !this.pathingCommand.isScheduled() || this.finished;
    }
}
