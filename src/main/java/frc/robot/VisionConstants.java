// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class VisionConstants {
  // Constants related to Limelight tracking

  public static final class LimelightMount {
    public static final double kCameraHeight = 0;
    public static final double kCameraAngleRad =
        0 * 3.14159 / 180.0; // Input angle of camera in Deg, will transform to Rad for equation
  }
  // Constants specifically for Limelight mounted for use with Coral Station
  public static final class CoralStationConstants {
    public static final double kAlignedSetpoint = 0;
    public static final double kDistanceSetpoint = 0;
    public static final double kMountAngleDegrees = 0;
    public static final double kLensHeightInches = 0;
    public static double coralStationActualDistance;
    public static final double kTargetHeight = 0;
  }
  // Constants specifically for Limelight mounted lefthand side from robot pov for Reef use
  public static final class LeftReefConstants {
    public static final double kAlignedSetpoint = 0;
    public static final double kDistanceSetpoint = 0;
    public static final double kMountAngleDegrees = 0;
    public static final double kLensHeightInches = 0;
    public static double leftReefActuaDistance;
    public static final double kTargetHeight = 0;
  }
  // Constants specifically for Limelight mounted righthand side from robot pov for Reef use
  public static final class RightReefConstants {
    public static final double kAlignedSetpoint = 0;
    public static final double kDistanceSetpoint = 0;
    public static final double kMountAngleDegrees = 0;
    public static final double kLensHeightInches = 0;
    public static double rightReefActuaDistance;
    public static final double kTargetHeight = 0;
  }

  public static final class AlignmentConstants {
    public static final double kP = .01;
    public static final double kI = 0;
    public static final double kD = 0;
    public static double driveTrackingAlignment = 0;
  }

  public static final class DistanceConstants {
    public static final double kP = .01;
    public static final double kI = 0;
    public static final double kD = 0;
    public static double driveTrackingDistance = 0;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
