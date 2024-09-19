// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.midi;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MidiSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile MidiSubsystem instance;
    private static Object mutex = new Object();

    public static MidiSubsystem getInstance() {
        MidiSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new MidiSubsystem();
                }
            }
        }
        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    private MidiSubsystem() {
        super("MidiSubsystem");
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}
}
