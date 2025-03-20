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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // To the future programmer seeing this: Don't put mutable variables in constants!!!
  // This is very bad practice!!
  //Oops! I made 2 more!!
  public static Rotation2d lastValidTargetAngle = Rotation2d.fromDegrees(0);

  // Controller Buttons//Controller buttons
  public static final Joystick leftController = new Joystick(1);
  public static final Joystick rightController = new Joystick(0);
  public static final Joystick buttonBox = new Joystick(2);
  // Top of reef (from alliance wall)
  public static final boolean one = buttonBox.getRawButton(1);
  // top right side
  public static final boolean two = buttonBox.getRawButton(2);
  // bottom right side
  public static final boolean three = buttonBox.getRawButton(3);
  // bottom of reef (from alliance wall)
  public static final boolean four = buttonBox.getRawButton(4);
  // bottom left
  public static final boolean five = buttonBox.getRawButton(5);
  // top left
  public static final boolean six = buttonBox.getRawButton(6);
  // left feed station (from alliance wall)
  public static final boolean seven = buttonBox.getRawButton(7);
  // right feed station (from alliance wall)
  public static final boolean eight = buttonBox.getRawButton(8);
  // processor
  public static final boolean nine = buttonBox.getRawButton(9);

  // added down

  public static final class CoralSubsystemConstants {
    public static final int kElevatorMotorCanId = 31;
    public static final int kArmMotorCanId = 34;
    public static final int kIntakeMotorCanId = 35;

    public static final class ElevatorSetpoints {
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 0;
      public static final int kLevel3 = 109;
      public static final int kLevel4 = 109;
    }

    public static final class ArmSetpoints {
      public static final double kFeederStation = -12.33;
      public static final double kLevel1 = -45;
      public static final double kLevel2 = -45;
      public static final double kLevel3 = -45;
      public static final double kLevel4 = -45;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
    }
  }

  public static final class AlgaeSubsystemConstants {
    public static final int kIntakeMotorCanId = 33;
    public static final int kPivotMotorCanId = 32;

    public static final class ArmSetpoints {
      public static final double kStow = -4.178;
      public static final double kHold = -6.203;
      public static final double kDown = -17.2;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
      public static final double kHold = 0.25;
    }
  }

  // added up

  // public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  // public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0,
  // Units.inchesToMeters(8)), ROBOT_MASS);
  // public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  // public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.75;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  // more added down

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  // added up

}
