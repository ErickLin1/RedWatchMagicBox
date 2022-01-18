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

    public static final int kDriverControllerPort = 1;
    public static final boolean kLeftReversedDefault = true;
    public static final boolean kRightReversedDefault = !kLeftReversedDefault;
    public static final int STALL_LIMIT = 45;
    public static final String kShuffleboardTab = "Testing";
    public static final int kCurrentLimit = 60;

    // Spark Maxes
    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 0;
    public static final int LEFT_MOTOR2_ID = 0; // Not in use
    public static final int RIGHT_MOTOR2_ID = 0; // Not in use

    // Talons
    public static final int kLeftTalonPort = 0;
    public static final int kRightTalonPort = 0;

    // Pneumatics
    public static final PneumaticsModuleType PneumaticType = PneumaticsModuleType.CTREPCM;
    public static final int SOL_PICKUP_PORT = 0;
    public static final int SOL_PUNCH_PORT = 0;
    public static final int SOL_HAB_PORT = 0;
    public static final int SOL_ARM_PORT = 0;
}
