// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.midi;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.MidiConstants;

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

    private Orchestra orchestra = new Orchestra();

    /** Creates a new MidiSubsystem. */
    private MidiSubsystem() {
        super("MidiSubsystem");

        orchestra.addInstrument(new TalonFX(MidiConstants.INSTRUMENT_1_ID), 0);
        orchestra.addInstrument(new TalonFX(MidiConstants.INSTRUMENT_2_ID), 1);
    }

    /**
     * Play a song with the orchestra.
     * @param chrpPath Path to the chrp file.
     */
    public void playSong(String chrpPath){
        orchestra.loadMusic(chrpPath);
        orchestra.play();
    }

    /** Stop current song */
    public void stopSong(){
        orchestra.stop();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}
}