// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile ShooterSubsystem instance;
    private static Object mutex = new Object();

    public static ShooterSubsystem getInstance() {
        ShooterSubsystem result = instance;

        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new ShooterSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX topShooterMotor = new TalonFX(ShooterConstants.TOP_SHOOTER_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);

    /** Creates a new IntakeSubsystem. */
    private ShooterSubsystem() {
        super("ShooterSubsystem");

        bottomShooterMotor.setControl(new Follower(ShooterConstants.TOP_SHOOTER_MOTOR_ID, false));
    }

    /**
     * Set the speed of the intake motors.
     * @param speed - Between -1.0 and 1.0.
     */
    public void setShooterSpeed(double speed) {
        topShooterMotor.set(speed);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}
}
