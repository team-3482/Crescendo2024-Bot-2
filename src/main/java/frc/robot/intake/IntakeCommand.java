// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.IntakeConstants;

/** Spins the intake motor. */
public class SpinIntakeCommand extends Command {
    /**
     * Creates a new IntakeCommand.
     */
    public IntakeCommand() {
        setName("IntakeCommand");
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setIntakeSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}