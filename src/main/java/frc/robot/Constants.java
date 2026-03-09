// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.PerUnit;

/** Add your docs here. */
public class Constants {

    public static class DriveTrainConstants {

        public static final int FLDrive = 2;
        public static final int BLDrive = 3;
        public static final int FRDrive = 4;
        public static final int BRDrive = 5;

    }

    public static class ShooterConstants {
        public static final int INDEXER = 7;
        public static final int ELEVATOR = 8;
        public static final int FLYWHEEL = 9;

        public static final int[] INDEXERLIMITS = {20, 5, 50};
    }
}
