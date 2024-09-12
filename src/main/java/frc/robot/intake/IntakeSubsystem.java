// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile IntakeSubsystem instance;
    private static Object mutex = new Object();

    public static IntakeSubsystem getInstance() {
        IntakeSubsystem result = instance;

        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new IntakeSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);

    /** Creates a new IntakeSubsystem. */
    private IntakeSubsystem() {
        super("IntakeSubsystem");
    }

    /**
     * Set the speed of the intake motors.
     * @param speed - Between -1.0 and 1.0.
     */
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}
}
