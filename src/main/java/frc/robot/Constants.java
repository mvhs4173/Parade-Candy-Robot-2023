// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double launchDelay = 1.0; // (seconds) Amount of time to wait (and flash LEDs) before launching once the button is pressed
    // Swerve Drive
    public static final double DISTANCE_BETWEEN_WHEELS_X = 16.5 / 12, // (feet)
                               DISTANCE_BETWEEN_WHEELS_Y = 12.5 / 12; // (feet)
    public static final Translation2d FR_LOCATION = new Translation2d(Units.feetToMeters(DISTANCE_BETWEEN_WHEELS_X / 2), Units.feetToMeters(DISTANCE_BETWEEN_WHEELS_Y / 2)),
                                      FL_LOCATION = new Translation2d(Units.feetToMeters(-DISTANCE_BETWEEN_WHEELS_X / 2), Units.feetToMeters(DISTANCE_BETWEEN_WHEELS_Y / 2)),
                                      BL_LOCATION = new Translation2d(Units.feetToMeters(-DISTANCE_BETWEEN_WHEELS_X / 2), Units.feetToMeters(-DISTANCE_BETWEEN_WHEELS_Y / 2)),
                                      BR_LOCATION = new Translation2d(Units.feetToMeters(DISTANCE_BETWEEN_WHEELS_X / 2), Units.feetToMeters(-DISTANCE_BETWEEN_WHEELS_Y / 2));
    public static final double FR_SWIVEL_ZERO_ANGLE = -90 + 65.5 + 180, // (degrees) The angle the encoder reads at while positioned such that it should read 0
                               FL_SWIVEL_ZERO_ANGLE = 0 - 113 + 180,
                               BL_SWIVEL_ZERO_ANGLE = 180 - 30 + 180,
                               BR_SWIVEL_ZERO_ANGLE = 180 + 120.5 + 180 + 90 + 45;
    public static final double FR_DRIVE_MULTIPLIER = -1, // Used to compensate for nuances in drive motor directions or percent speeds. One side (left or right) naturally goes backwards, due to the way the gearbox works
                               FL_DRIVE_MULTIPLIER = 1,
                               BL_DRIVE_MULTIPLIER = 1,
                               BR_DRIVE_MULTIPLIER = -1;
}
