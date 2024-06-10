// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Positions.PositionInitialization;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;

public class RobotContainer {
    // Thread-safe singleton design pattern.
    private static volatile RobotContainer instance;
    private static Object mutex = new Object();


    public static RobotContainer getInstance() {
        RobotContainer result = instance;
       
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new RobotContainer();
                }
            }
        }
        return instance;
    }

    private final SendableChooser<Command> autoChooser;
    // TODO : Move to SwerveSubsystem once it exists
    private final SendableChooser<PositionInitialization> positionChooser = new SendableChooser<PositionInitialization>();
    private final ShuffleboardLayout layout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT).getLayout("SwerveSubsystem", BuiltInLayouts.kList);

    // Instance of the controllers used to drive the robot
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    /** Creates an instance of the robot controller */
    public RobotContainer() {
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

        initializeSubsystems();
        // Register named commands for pathplanner (always do this after subsystem initialization)
        registerNamedCommands();

        configureDriverBindings();
        configureOperatorBindings();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        // Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        //     .add("Auto Chooser", autoChooser)
        //     .withWidget(BuiltInWidgets.kComboBoxChooser)
        //     .withPosition(11, 0)
        //     .withSize(4, 1);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /** Creates instances of each subsystem so periodic runs */
    private void initializeSubsystems() {
        // TODO : Move this to SwerveSubsystem once it's created

        for (PositionInitialization position : PositionInitialization.values()) {
            this.positionChooser.addOption(position.name(), position);
            if (position == PositionInitialization.LIMELIGHT) {
                this.positionChooser.setDefaultOption(position.name(), position);
            }
        }
        
        this.positionChooser.onChange((PositionInitialization position) -> {}
            // SwerveSubsystem.getInstance().setPose(Positions.getStartingPose(position))
        );

        this.layout.add("Starting Position", this.positionChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
        // Re-set chosen position.
        this.layout.add("Set Starting Position",
            Commands.runOnce(() -> {}
                // SwerveSubsystem.getInstance().setPose(Positions.getStartingPose(this.positionChooser.getSelected()))
            ).ignoringDisable(true).withName("Set Again"))
            .withWidget(BuiltInWidgets.kCommand);
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        
    }

    /** Configures the button bindings of the driver controller */
    private void configureDriverBindings() {
        // Cancel all currently scheduled commands
        driverController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));
    }

    /** Configures the button bindings of the driver controller */
    private void configureOperatorBindings() {
        // Unused while building and testing new kraken swerve drive
        
        // Cancel all currently scheduled commands
        operatorController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}