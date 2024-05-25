// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A class that stores positions used for initialization and calculations.
 */
public final class Positions {
    // TODO : redo this file, because it's a freaking mess

    /** SPEAKER positions to target */
    public static final Map<DriverStation.Alliance, Translation3d> SPEAKER_TARGETS = Map.ofEntries(
        Map.entry(DriverStation.Alliance.Blue, new Translation3d(0, 5.55, 3)),
        Map.entry(DriverStation.Alliance.Red, new Translation3d(16.5, 5.55, 3))
    );

    /** Starting positions using the PathPlanner field as reference */
    public static enum StartingPositions {
        TOP("Top", 3),
        MIDDLE("Middle", 2),
        BOTTOM("Bottom", 1),
        AUTO("Auto", 0)
        ;

        String name;
        int location;

        private StartingPositions(String name, int position) {
            this.name = name;
            this.location = position;
        }
        /**
         * Gets the human-readable name of the position.
         * @return the name.
         */
        public String getName() {
            return this.name;
        }
        /**
         * Gets the integer location based on {@link DriverStation#getLocation()}.
         * @return the location.
         */
        public int getLocation() {
            return this.location;
        }
        /**
         * Gets an array of the {@link StartingPositions}.
         * @return the array.
         */
        public static StartingPositions[] getStartingPositions() {
            return new StartingPositions[]{TOP, MIDDLE, BOTTOM, AUTO};
        }
    }

    /** Initial bot positions used for initializing odometry, blue-alliance relative. */
    public static final Map<DriverStation.Alliance, Map<Integer, Pose2d>> STARTING_POSITIONS = Map.ofEntries(
        Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
            Map.entry(3, new Pose2d(new Translation2d(0.75, 6.66), Rotation2d.fromDegrees(60))),
            Map.entry(2, new Pose2d(new Translation2d(1.34, 5.55), Rotation2d.fromDegrees(0))),
            Map.entry(1, new Pose2d(new Translation2d(0.75, 4.45), Rotation2d.fromDegrees(300))))),
        Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
            Map.entry(3, new Pose2d(new Translation2d(15.8, 6.66), Rotation2d.fromDegrees(300))),
            Map.entry(2, new Pose2d(new Translation2d(15.2, 5.55), Rotation2d.fromDegrees(180))),
            Map.entry(1, new Pose2d(new Translation2d(15.8, 4.50), Rotation2d.fromDegrees(60)))))
    );

    /** Names used to organize pathfinding positions in {@link Positions#PATHFIND_POSITIONS}. */
    public static enum PathfindingPosition {
        SPEAKER_TOP,
        SPEAKER_MIDDLE,
        SPEAKER_BOTTOM,
        AMP
    }

    /** Position the robot should line up to, blue-alliance relative. */
    public static final Map<DriverStation.Alliance, Map<PathfindingPosition, Pose2d>> PATHFIND_POSITIONS = Map.ofEntries(
        Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
            Map.entry(PathfindingPosition.SPEAKER_TOP, STARTING_POSITIONS.get(DriverStation.Alliance.Blue).get(StartingPositions.TOP.getLocation())),
            Map.entry(PathfindingPosition.SPEAKER_MIDDLE, STARTING_POSITIONS.get(DriverStation.Alliance.Blue).get(StartingPositions.MIDDLE.getLocation())),
            Map.entry(PathfindingPosition.SPEAKER_BOTTOM, STARTING_POSITIONS.get(DriverStation.Alliance.Blue).get(StartingPositions.BOTTOM.getLocation())),
            Map.entry(PathfindingPosition.AMP, new Pose2d(new Translation2d(1.8, 7.66), Rotation2d.fromDegrees(90)))
        )),
        Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
            Map.entry(PathfindingPosition.SPEAKER_TOP, STARTING_POSITIONS.get(DriverStation.Alliance.Red).get(StartingPositions.TOP.getLocation())),
            Map.entry(PathfindingPosition.SPEAKER_MIDDLE, STARTING_POSITIONS.get(DriverStation.Alliance.Red).get(StartingPositions.MIDDLE.getLocation())),
            Map.entry(PathfindingPosition.SPEAKER_BOTTOM, STARTING_POSITIONS.get(DriverStation.Alliance.Red).get(StartingPositions.BOTTOM.getLocation())),
            Map.entry(PathfindingPosition.AMP, new Pose2d(new Translation2d(14.7, 7.66), Rotation2d.fromDegrees(-90)))
        ))
    );
}
