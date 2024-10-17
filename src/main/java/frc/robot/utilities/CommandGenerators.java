package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.intake.RunIntakeCommand;
import frc.robot.pivot.PivotCommand;

public class CommandGenerators {
    /**
     * Creates a new sequenced command to pivot shooter
     * down to the hardstop and spin the intake motors.
     * @return The command.
     */
    public static Command getIntakeCommand() {
        return Commands.sequence(
                new PivotCommand(PivotConstants.ABOVE_LIMELIGHT_ANGLE),
                new RunIntakeCommand()
        );
    }
}