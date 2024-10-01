// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.constants.PhysicalConstants.ShooterConstants.ShooterSlot0Gains;
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
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

    /** Creates a new ShooterSubsystem. */
    private ShooterSubsystem() {
        super("ShooterSubsystem");

        configureMotionMagic();

        // 20 ms update frequency (1 robot cycle)
        this.topShooterMotor.getVelocity().setUpdateFrequency(50);

        this.bottomShooterMotor.setControl(new Follower(ShooterConstants.TOP_SHOOTER_MOTOR_ID, true));
    }

    /**
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotionMagic() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = ShooterConstants.ROTOR_TO_MECHANISM_RATIO; 

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kG = ShooterSlot0Gains.kG;
        slot0Configs.kS = ShooterSlot0Gains.kS;
        slot0Configs.kV = ShooterSlot0Gains.kV;
        slot0Configs.kA = ShooterSlot0Gains.kA;
        slot0Configs.kP = ShooterSlot0Gains.kP;
        slot0Configs.kI = ShooterSlot0Gains.kI;
        slot0Configs.kD = ShooterSlot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ShooterConstants.JERK;

        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Top motor not inverted.
        this.topShooterMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Bottom motor inverted.
        this.bottomShooterMotor.getConfigurator().apply(configuration);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        System.out.printf("Velocity : %f rot/s%n", getVelocity());
    }

    /**
     * Aims for a velocity using MotionMagicVelocity.
     * @param speed - In rotations/sec.
     */
    public void motionMagicVelocity(double speed) {
        MotionMagicVelocityVoltage control = motionMagicVelocityVoltage
            .withSlot(0)    
            .withVelocity(speed);
        
        this.topShooterMotor.setControl(control);
    }

    /**
     * Sets the motors to a speed.
     * @param speed - Between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        this.topShooterMotor.set(speed);
    }

    /**
     * Gets the mechanism velocity of the top motor.
     * @return The velocity in rot/s.
     */
    public double getVelocity() {
        return this.topShooterMotor.getVelocity().getValueAsDouble();
    }
}
