// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile PivotSubsystem instance;
    private static Object mutex = new Object();

    public static PivotSubsystem getInstance() {
        PivotSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new PivotSubsystem();
                }
            }
        }
        return instance;
    }

    /** Creates a new PivotSubsystem. */
    private PivotSubsystem() {
        super("PivotSubsystem");
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}
}
