// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;

/** An Command that pivots the shooter to a position. */
public class PivotCommand extends Command {
    private double position;

    /**
     * Creates a new ExampleCommand.
     * @param position - The position in degrees.
     */
    public PivotCommand(double position) {
        setName("PivotCommand");
        
        this.position = position;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

// TODO