// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.midi;

import edu.wpi.first.wpilibj2.command.Command;

/** Play a midi file. */
public class MidiCommand extends Command {
    private String chrpPath;

    /**
     * * Creates a new MidiCommand.
     * @param chrpPath Path to the chrp file.
    */
    public MidiCommand(String chrpPath) {
        setName("MidiCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(MidiSubsystem.getInstance());

        this.chrpPath = chrpPath;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        MidiSubsystem.getInstance().setChrpPath(chrpPath);

        MidiSubsystem.getInstance().playSong();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        MidiSubsystem.getInstance().stopSong();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !MidiSubsystem.getInstance().getIsPlaying();
    }
}