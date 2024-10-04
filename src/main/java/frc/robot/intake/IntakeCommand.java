// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.IntakeConstants;

/** Runs the intake for a Note. */
public class IntakeCommand extends Command {
    private double velocity;

    /**
     * Creates a new IntakeCommand.
     * @param velocity - The speed at which the intake should run in rot/s.
     * @apiNote This value is clamped by {@link IntakeConstants#CRUISE_SPEED}.
     */
    public IntakeCommand(double velocity) {
        setName("IntakeCommand");

        this.velocity = velocity;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().motionMagicVelocity(this.velocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return IntakeSubsystem.getInstance().hasNote();
    }
}
