// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.PivotConstants;

/**
 * A command that moves the pivot down until the velocity is 0
 * (assumes this phenomenon to be a hard stop)
 * and resets the position to that angle
 * (after a 0.5 second delay to allow the pivot to relax).
 */
public class ResetAtHardstopCommand extends Command {
    private boolean nearCurrentStop;
    private Timer timer;
    private boolean hasMovedDown;
    
    /**
     * Creates a new ExampleCommand.
     * @param nearCurrentStop - Whether or not to only update the hard stop position
     * if it is within 3 degrees of the believed position.
     */
    public ResetAtHardstopCommand(boolean nearCurrentStop) {
        setName("ResetAtHardstopCommand");
        
        this.nearCurrentStop = nearCurrentStop;
        this.timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(PivotSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.timer.restart();
        this.hasMovedDown = false;
        
        PivotSubsystem.getInstance().setPivotSpeed(-0.1, false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Active MotionMagic might cause it to jerk up for a little bit
        if (!this.hasMovedDown && PivotSubsystem.getInstance().getVelocity() >= 0) {
            PivotSubsystem.getInstance().setPivotSpeed(-0.1);
        }
        
        else if (!this.hasMovedDown && PivotSubsystem.getInstance().getVelocity() < 0
        ) {
            this.hasMovedDown = true;
            this.timer.stop();
            this.timer.reset();
        }

        else if (this.timer.get() == 0 && PivotSubsystem.getInstance().getVelocity() == 0) {
            PivotSubsystem.getInstance().setPivotSpeed(0.05);
            this.timer.restart();
        }
        
        else if (this.timer.hasElapsed(0.1)) {
            PivotSubsystem.getInstance().setPivotSpeed(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        PivotSubsystem.getInstance().setPivotSpeed(0);
        this.timer.stop();

        if (!interrupted) { // If interrupted, assume it probably isn't at the hard stop
            double difference = PivotConstants.LOWER_HARD_STOP - PivotSubsystem.getInstance().getPosition();
            if (!nearCurrentStop || Math.abs(difference) <= 3) {
                PivotSubsystem.getInstance().setPositionHardStop();
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(0.5);
    }
}
