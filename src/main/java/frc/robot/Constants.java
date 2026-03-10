// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

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
        public static final int[] INDEXERCURRANTLIMITS = {20, 10, 50}; // Stall current, free current, stall currant rpm
        public static final int[] ELEVATORCURRANTLIMITS = {20, 5, 50}; // Stall current, free current, stall currant rpm

        public static final boolean INTAKEINVERT = true;
        public static final boolean INDEXERINVERT = false;
        public static final boolean ELEVATORINVERT = true;
        public static final InvertedValue FLYWHEELINVERT = InvertedValue.Clockwise_Positive;



        public static final double INTAKESPEED = 1;
        public static final double INTAKEAGITATESPEED = 0.25;
        public static final double INDEXERSPEED = 0.5;
        public static final double ELEVATORSPEED = 1;
    }
}
