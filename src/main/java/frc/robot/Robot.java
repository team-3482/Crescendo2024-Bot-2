// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command auton;
    
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // TODO LL
        // Port forward all required LL ports. Necessary for robot connections over ethernet.
        // for (int port = 5800; port <= 5807; port++) {
        //     PortForwarder.add(port, LimelightConstants.INTAKE_LLIGHT + ".local", port);
        //     PortForwarder.add(port + 10, LimelightConstants.SHOOTER_LLIGHT + ".local", port);
        // }

        // Initialize RobotContainer and all subsystems
        RobotContainer.getInstance();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // TODO implement LL using this stub
        /**
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        // if (UseLimelight) {
        //   var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

        //   Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

        //   if (lastResult.valid) {
        //     m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
        //   }
        // }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        this.auton = RobotContainer.getInstance().getAutonomousCommand();
        if (this.auton != null) {
            try {
                this.auton.schedule();
            }
            catch (Exception error) {
                error.printStackTrace();
            }
        }
        else {
            System.err.println("No auton command found.");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (this.auton != null) {
            this.auton.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
