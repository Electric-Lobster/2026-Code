// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Utilities.LookupTable;

/** Add your docs here. */
public class Constants {

    public static class DriveTrainConstants {

        public static final int FLDrive = 2;
        public static final int BLDrive = 3;
        public static final int FRDrive = 4;
        public static final int BRDrive = 5;

    }

    public static class BallSystemConstants {
        public static final int INTAKE = 6;
        public static final int INDEXER = 7;
        public static final int ELEVATOR = 8;
        public static final int FLYWHEEL = 9;

        public static final int[] INTAKECURRANTLIMITS = {60, 40, 50}; // Stall current, free current, stall currant rpm
        public static final int[] INDEXERCURRANTLIMITS = {60, 40, 50}; // Stall current, free current, stall currant rpm
        public static final int[] ELEVATORCURRANTLIMITS = {20, 5, 50}; // Stall current, free current, stall currant rpm

        public static final boolean INTAKEINVERT = true;
        public static final boolean INDEXERINVERT = false;
        public static final boolean ELEVATORINVERT = true;
        public static final InvertedValue FLYWHEELINVERT = InvertedValue.Clockwise_Positive;



        public static final double INTAKESPEED = 1;
        public static final double INTAKEREVERSESPEED = -0.5;
        public static final double INTAKEAGITATESPEED = 0.25;
        public static final double INDEXERSPEED = 0.5;
        public static final double INDEXERREVERSESPEED = -0.5;
        public static final double ELEVATORSPEED = 1;
    }

    public static class ShooterConstants {

        public static final Pose2d REDGOAL = new Pose2d(11.92, 4.033, new Rotation2d());
        public static final Pose2d BLUEGOAL = new Pose2d(4.625, 4.033, new Rotation2d());

        public static final Transform2d SHOOTEROFFSET = new Transform2d(-0.2, 0.2, new Rotation2d(0));
    
        private static final double[] TABLEDATA = {
            //Distance, Speed
            1.0, 48,
            1.25, 48.07,
            2, 52.19,
            2.5, 54.28,
            3, 58.08,
            3.5, 62.05,
            3.75, 62.22,
            4, 64.59,
            4.5, 67.58,
            5, 72.12,
            5.5, 74.86,
            6, 77.74
        };

        public static final LookupTable LOOKUPTABLE = new LookupTable(TABLEDATA);
    
    }
}
