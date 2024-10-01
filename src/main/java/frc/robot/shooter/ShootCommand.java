// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.intake.IntakeSubsystem;

/** Shoots a Note. */
public class ShootCommand extends Command {
    private double velocity;
    private boolean atVelocity;
    private boolean cancel;
    private Timer timer;

    /**
     * Creates a new ShootCommand.
     * @param velocity - The speed at which the shooting motors should run in rotations/sec.
     */
    public ShootCommand(double velocity) {
        setName("ShootCommand");
        
        this.velocity = velocity;
        this.timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance(), IntakeSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (IntakeSubsystem.getInstance().hasNote()) {
            this.cancel = false;
        }
        else {
            this.cancel = true;
            return;
        }

        this.atVelocity = false;
        this.timer.reset();

        ShooterSubsystem.getInstance().motionMagicVelocity(this.velocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.cancel) return;

        if (!this.atVelocity
            && Math.abs(
                ShooterSubsystem.getInstance().getVelocity() - this.velocity
            ) <= ShooterConstants.VELOCITY_TOLERANCE
        ) {
            this.atVelocity = true;
        }

        if (this.atVelocity) {
            IntakeSubsystem.getInstance().setSpeed(IntakeConstants.INTAKE_SPEED);
            this.timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setSpeed(0);
        IntakeSubsystem.getInstance().setSpeed(0);

        this.timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.cancel || this.timer.hasElapsed(0.25);
    }
}
