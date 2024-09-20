// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

/** Shoots a Note. */
public class ShootCommand extends Command {
    private double speed;

    /**
     * Creates a new ShootCommand.
     * @param speed - The speed at which the shooting motors should run.
     */
    public ShootCommand(double speed) {
        setName("ShootCommand");
        
        this.speed = speed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setShooterSpeed(this.speed);
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
