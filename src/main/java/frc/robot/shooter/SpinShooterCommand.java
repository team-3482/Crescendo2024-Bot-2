// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.shooter.ShooterSubsystem;

/** Spins the shooter motors. */
public class SpinShooterCommand extends Command {
    /**
     * Creates a new SpinShooterCommand.
     */
    public SpinShooterCommand() {
        setName("SpinShooterCommand");
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setShooterSpeed(ShooterConstants.SHOOTER_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShooterSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
