// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Controller Ports
    public static class ControllerConstants {
        public static final int kSparkControllerPort = 1;
        public static final int kTalonControllerPort = 2;
    }

    public static class ControlPanelConstants {
        public static final String kShuffleboardTab = "Magic Panel";
    }

    public static class DriveSpark {
        public static final boolean kLeftReversedDefault = true;
        public static final boolean kRightReversedDefault = !kLeftReversedDefault;
        public static final int STALL_LIMIT = 45;
        public static final int kCurrentLimit = 60;

        // Spark Maxes
        public static final int LEFT_SPARK_ID = 3;
        // public static final int RIGHT_SPARK_ID = 6;
        public static final int maxSparkRPM = 5500;
        // public static final String kShuffleboardTabSpark = "Spark Maxes";
    }

    public static final class TelescopingConstants {

		public static final int kArmExtendPort = 1;
		public static final int kArmPivotPort = 10;
        // pivoting gearbox = 1:125 
        public static final double kTelescopingGearRatio = 1.0 / 16.0;
        public static final double kPivotingGearRatio = 1.0 / 125.0;

        public static final double kAnglePerRevolution = kPivotingGearRatio * 3.14;
        public static final double kDistancePerRevolution = kTelescopingGearRatio * (7.0 / 8.0) * 3.14; //TODO: put in gear ratio for the climbers!!!
        public static final double kArmSize = 12.0;
    }

    public static class DriveTalon {
        // Talons
        public static final int kLeftTalonPort = 11;
        public static final int kRightTalonPort = 10;
        // public static final String kShuffleboardTabTalon = "Talons";
    }

    public static class PneumaticSolenoid {
        // Pneumatics
        public static final PneumaticsModuleType PneumaticType = PneumaticsModuleType.CTREPCM;
        public static final int solMotorPort = 4;
        public static final int solMotorPort2 = 6;
    }

    public static class BeambreakConstants {
        public static final int BeambreakPort = 1;
    }
}
