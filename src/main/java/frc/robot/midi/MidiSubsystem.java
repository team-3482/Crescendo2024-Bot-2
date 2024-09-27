// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.midi;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.swerve.TunerConstants;

public class MidiSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern. 
    private static volatile MidiSubsystem instance;
    private static Object mutex = new Object();
    private static final int[] orchestraDevices = { 5, 6, 7, 8, 11, 12, 11, 12, 13 };
    private String chrpPath;

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

        for (int i = 0; i < orchestraDevices.length; i++) {
            orchestra.addInstrument(new TalonFX(orchestraDevices[i], RobotConstants.CTRE_CAN_BUS));
        }

        orchestra.loadMusic(chrpPath);
    }

    /** Set the path of the chrp file. */
    public void setChrpPath(String chrpPath) {
        this.chrpPath = chrpPath;
    }

    /** Start playing the song */
    public void playSong() {
        orchestra.play();
    }

    /** Stop current song */
    public void stopSong() {
        orchestra.stop();
    }

    public boolean getIsPlaying() {
        return orchestra.isPlaying();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}
}