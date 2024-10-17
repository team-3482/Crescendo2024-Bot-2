// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

/**
 * Constants used throughout the code that are not categorized in other constants files.
 */
public final class Constants {
    /** Constants used for maintaining a heading when using {@link SwerveRequest#FieldCentricFacingAngle} */
    public static final PhoenixPIDController HeadingControllerFacingAngle = new PhoenixPIDController(5.5, 0, 0.1);
    
    /**
     * Tab names in Shuffleboard.
     */
    public static final class ShuffleboardTabNames {
        public static final String DEFAULT = "Competition";
        public static final String UTILITIES = "Utilities";
    }

    /** Constants for the controller and any controller related assignments. */
    public static final class ControllerConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;
        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        /** Whether or not to accept directional pad input for movement. */
        public static final boolean DPAD_DRIVE_INPUT = true;
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;
    }

    /** Constants used for in-game behavior. */
    public static final class BehaviorConstants {
        /** How close to the target rotation to be before printing that it is ready to shoot .*/
        public static final double FACING_ANGLE_TOLERANCE = 2;
        
        /** The position of the pivot in degrees to shoot into the speaker from right in front of it. */
        public static final double PIVOT_POSITION_SPEAKER = 50;
    }
}
