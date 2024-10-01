// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;

/** Runs the intake for a Note. */
public class IntakeCommand extends Command {
    private double speed;

    /**
     * Creates a new IntakeCommand.
     * @param speed - The speed at which the intake motors should run.
     */
    public IntakeCommand(double speed) {
        setName("IntakeCommand");

        this.speed = speed;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setSpeed(this.speed);
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
        return false; // TODO LATER : Stopping logic with laser
    }
}
