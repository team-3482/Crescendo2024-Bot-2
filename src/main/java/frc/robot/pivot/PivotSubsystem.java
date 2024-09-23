// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants.PivotSlot0Gains;

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

    private TalonFX leftPivotMotor = new TalonFX(PivotConstants.LEFT_PIVOT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX rightPivotMotor = new TalonFX(PivotConstants.RIGHT_PIVOT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    /** Creates a new PivotSubsystem. */
    private PivotSubsystem() {
        super("PivotSubsystem");

        configureMotors();
        setPositionHardStop();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        System.out.printf("position : %f ; rotor : %f%n", getPosition(), this.rightPivotMotor.getRotorPosition().getValueAsDouble());
    }

    /**
     * A helper method that configures MotionMagic and
     * sets the follower on both motors.
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = PivotConstants.ROTOR_TO_MECHANISM_RATIO; 

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.kS = PivotSlot0Gains.kS;
        slot0Configs.kV = PivotSlot0Gains.kV;
        slot0Configs.kA = PivotSlot0Gains.kA;
        slot0Configs.kP = PivotSlot0Gains.kP;
        slot0Configs.kI = PivotSlot0Gains.kI;
        slot0Configs.kD = PivotSlot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = PivotConstants.CRUISE_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = PivotConstants.MOTION_MAGIC_JERK;

        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Right motor not inverted.
        this.rightPivotMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Left motor inverted.
        this.leftPivotMotor.getConfigurator().apply(configuration);

        this.leftPivotMotor.setControl(new Follower(PivotConstants.RIGHT_PIVOT_MOTOR_ID, true));
    }

    /**
     * Goes to a position using Motion Magic slot 0.
     * @param position - The position for the pivot in degrees.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    public void motionMagicPosition(double position) {
        position = MathUtil.clamp(position, PivotConstants.LOWER_ANGLE_LIMIT, PivotConstants.UPPER_ANGLE_LIMIT);

        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(Units.degreesToRotations(position));
        
        this.rightPivotMotor.setControl(control);
    }

    /**
     * Sets the speed for the pivot motors.
     * @param speed - Speed from -1.0 to 1.0 for the motors.
     * @param safe - Whether or not to respect soft limits. See {@link PivotConstants} for limits.
     */
    public void setPivotSpeed(double speed, boolean safe) {
        if (safe && speed != 0) {
            double position = getPosition();
            if ((speed > 0 && position >= PivotConstants.UPPER_ANGLE_LIMIT)
                || (speed < 0 && position <= PivotConstants.LOWER_ANGLE_LIMIT)) {
                    speed = 0;
                }
        }

        this.rightPivotMotor.set(speed);
    }

    /**
     * Sets the speed for the pivot motors.
     * @param speed - Speed from -1.0 to 1.0 for the motors.
     * @apiNote Will respect soft limits marked in {@link PivotConstants}.
     */
    public void setPivotSpeed(double speed) {
        setPivotSpeed(speed, true);
    }

    /**
     * Sets the mechanism position of both motors.
     * @param position - The position in degrees.
     */
    public void setPosition(double position) {
        position = Units.degreesToRotations(position);
        this.rightPivotMotor.setPosition(position);
        this.leftPivotMotor.setPosition(position);
    }

    /**
     * Sets the mechanism position of both motors to the lower hard stop.
     * @apiNote See hard stop at {@link PivotConstants#LOWER_ANGLE_LIMIT}
     */
    public void setPositionHardStop() {
        setPosition(PivotConstants.LOWER_ANGLE_LIMIT);
    }

    /**
     * Gets the mechanism position of the motor.
     * @return The angle in degrees.
     */
    public double getPosition() {
        return Units.rotationsToDegrees(this.rightPivotMotor.getPosition().getValueAsDouble());
    }
}
