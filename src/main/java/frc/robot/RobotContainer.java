// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Positions.PositionInitialization;
import frc.robot.intake.IntakeCommand;
import frc.robot.pivot.ManuallyPivotCommand;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.pivot.ResetAtHardstopCommand;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.swerve.Telemetry;
import frc.robot.swerve.TunerConstants;
import frc.robot.vision.VisionSubsystem;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.Positions;

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
    // Position chooser
    private final SendableChooser<PositionInitialization> positionChooser = new SendableChooser<PositionInitialization>();
    private final ShuffleboardLayout layout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT).getLayout("SwerveSubsystem", BuiltInLayouts.kList);

    // Instance of the controllers used to drive the robot
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    /** Creates an instance of the robot controller */
    public RobotContainer() {
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

        configureDrivetrain(); // This is done separately because it works differently from other Subsystems

        initializeSubsystems();
        // Register named commands for pathplanner (always do this after subsystem initialization)
        registerNamedCommands();

        configureDriverBindings();
        configureOperatorBindings();

        this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        // Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        //     .add("Auto Chooser", autoChooser)
        //     .withWidget(BuiltInWidgets.kComboBoxChooser)
        //     .withPosition(11, 0)
        //     .withSize(4, 1);
        SmartDashboard.putData("Auto Chooser", this.autoChooser);
    }

    /**
     * This function initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the {@code Phoenix6 Swerve Example} that can be found on GitHub.
     */
    private void configureDrivetrain() {
        final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * ControllerConstants.DEADBAND).withRotationalDeadband(MaxAngularRate * ControllerConstants.DEADBAND) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
        
        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        final SwerveRequest.FieldCentric fieldCentricMove = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        final Telemetry logger = new Telemetry(MaxSpeed);

        Trigger rightBumper = driverController.rightBumper();

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive
                // Drive forward with negative Y (forward)
                .withVelocityX(-driverController.getLeftY() * MaxSpeed * (rightBumper.getAsBoolean() ? ControllerConstants.FINE_CONTROL_MULT : 1))
                // Drive left with negative X (left)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed * (rightBumper.getAsBoolean() ? ControllerConstants.FINE_CONTROL_MULT : 1))
                // Drive counterclockwise with negative X (left
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate * (rightBumper.getAsBoolean() ? ControllerConstants.FINE_CONTROL_MULT : 1))
            )
            .ignoringDisable(true)
        );

        driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.y().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
            new Rotation2d(
                Math.abs(driverController.getLeftY()) >= 0.25 ? -driverController.getLeftY() : 0,
                Math.abs(driverController.getLeftX()) >= 0.25 ? -driverController.getLeftX() : 0
            )
        )));

        // Burger
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        // Double Rectangle
        driverController.back().onTrue(drivetrain.runOnce(() -> {
            Pose2d estimatedPose = VisionSubsystem.getInstance().getEstimatedPose();
            if (!estimatedPose.equals(new Pose2d())) {
                drivetrain.seedFieldRelative(estimatedPose);
            }
        }));

        // This looks terrible, but I can't think of a better way to do it </3
        if (ControllerConstants.DPAD_DRIVE_INPUT) {
            /** POV angle : [X velocity, Y velocity] in m/s */
            final Map<Integer, Double[]> povSpeeds = Map.ofEntries(
                Map.entry(  0, new Double[]{ 0.25,  0.0}),
                Map.entry( 45, new Double[]{ 0.25, -0.25}),
                Map.entry( 90, new Double[]{ 0.0,  -0.25}),
                Map.entry(135, new Double[]{-0.25, -0.25}),
                Map.entry(180, new Double[]{-0.25,  0.0}),
                Map.entry(225, new Double[]{-0.25,  0.25}),
                Map.entry(270, new Double[]{ 0.0,   0.25}),
                Map.entry(315, new Double[]{ 0.25,  0.25})
            );

            povSpeeds.forEach(
                (Integer angle, Double[] speeds) -> driverController.pov(angle).whileTrue(
                    drivetrain.applyRequest(() -> fieldCentricMove.withVelocityX(speeds[0]).withVelocityY(speeds[1]))
                )
            );
        }
        
        drivetrain.registerTelemetry(logger::telemeterize);

        // Position chooser
        for (PositionInitialization position : PositionInitialization.values()) {
            this.positionChooser.addOption(position.name(), position);
            if (position == PositionInitialization.LIMELIGHT) {
                this.positionChooser.setDefaultOption(position.name(), position);
            }
        }
        
        this.positionChooser.onChange((PositionInitialization position) ->
            drivetrain.seedFieldRelative(Positions.getStartingPose(position))
        );

        this.layout.add("Starting Position", this.positionChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
        // Re-set chosen position.
        this.layout.add("Set Starting Position",
            Commands.runOnce(
                () -> drivetrain.seedFieldRelative(Positions.getStartingPose(this.positionChooser.getSelected()))
            ).ignoringDisable(true).withName("Set Again"))
            .withWidget(BuiltInWidgets.kCommand);
    }

    /** Creates instances of each subsystem so periodic runs */
    private void initializeSubsystems() {      
        VisionSubsystem.getInstance();

        PivotSubsystem.getInstance();
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        
    }

    /**
     * Configures the button bindings of the driver controller
     * @apiNote POV, joysticks, and start / back are all used in {@link RobotContainer#configureDrivetrain()}
     */
    private void configureDriverBindings() {
        // Cancel all currently scheduled commands
        driverController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));
    }

    /** Configures the button bindings of the driver controller */
    private void configureOperatorBindings() {
        // Cancel all currently scheduled commands
        operatorController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        PivotSubsystem.getInstance().setDefaultCommand(new ManuallyPivotCommand(
            () -> operatorController.getRightTriggerAxis(),
            () -> operatorController.getLeftTriggerAxis(),
            false
        ));

        operatorController.pov(270)
            .whileTrue(PivotSubsystem.getInstance().run(
                () -> PivotSubsystem.getInstance().motionMagicPosition(90)
            ));
        operatorController.pov(0)
            .whileTrue(PivotSubsystem.getInstance().run(
                () -> PivotSubsystem.getInstance().motionMagicPosition(70)
            ));
        operatorController.pov(90)
            .whileTrue(PivotSubsystem.getInstance().run(
                () -> PivotSubsystem.getInstance().motionMagicPosition(45)
            ));
        operatorController.pov(180)
            .whileTrue(PivotSubsystem.getInstance().run(
                () -> PivotSubsystem.getInstance().motionMagicPosition(5)
            ));
        operatorController.a().onTrue(new ResetAtHardstopCommand(true));
        
        operatorController.x().whileTrue(new IntakeCommand(IntakeConstants.INTAKE_SPEED));
        operatorController.y().whileTrue(new IntakeCommand(-IntakeConstants.INTAKE_SPEED));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test1");
        // return autoChooser.getSelected();
    }
}