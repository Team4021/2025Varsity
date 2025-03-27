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

import static frc.robot.subsystems.vision.VisionConstants.camera0;
import static frc.robot.subsystems.vision.VisionConstants.camera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  private final Drive drive;
  private final Vision vision;

  // Controller
  public static final Joystick leftController = new Joystick(1);
  public static final Joystick rightController = new Joystick(0);
  public static final Joystick buttonBox = new Joystick(2);

  // Drive suppliers
  DoubleSupplier driverX = () -> -leftController.getRawAxis(1); // Y-axis joystick
  DoubleSupplier driverY = () -> -leftController.getRawAxis(0); // X-axis joystick
  DoubleSupplier angleX = () -> rightController.getRawAxis(0); // X-axis joystick
  DoubleSupplier angleY = () -> -rightController.getRawAxis(1); // Y-axis joystick

  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera1, drive::getRotation),
                new VisionIOLimelight(camera0, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }
    NamedCommands.registerCommand(
        "Algae Clear", m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeClear));

    NamedCommands.registerCommand(
        "Algae Clear Two", m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeClear2));

    NamedCommands.registerCommand(
        "Algae Knockout", m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeKnockout));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();

    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // Joystick drive command (driver and operator)
    Supplier<Command> joystickDriveCommandFactory =
        () ->
            DriveCommands.joystickDriveAtAngle(
                drive, driverX, driverY, this::getTargetAngleFromJoystick);
    drive.setDefaultCommand(joystickDriveCommandFactory.get());

    // Note to self: removed negatives on x axis

    new JoystickButton(buttonBox, 1)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(0);
                },
                drive));
    new JoystickButton(buttonBox, 2)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(300);
                },
                drive));

    new JoystickButton(buttonBox, 3)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(240);
                },
                drive));

    new JoystickButton(buttonBox, 4)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(180);
                },
                drive));

    new JoystickButton(buttonBox, 5)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(120);
                },
                drive));

    new JoystickButton(buttonBox, 6)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(60);
                },
                drive));

    new JoystickButton(buttonBox, 7)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(155);
                },
                drive));

    new JoystickButton(buttonBox, 8)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(335);
                },
                drive));

    new JoystickButton(buttonBox, 9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(270);
                },
                drive));

    new JoystickButton(buttonBox, 10)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Constants.lastValidTargetAngle = Rotation2d.fromDegrees(90);
                },
                drive));

    new JoystickButton(rightController, 2)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeClear));

    new JoystickButton(rightController, 3)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeClear2));

    new JoystickButton(rightController, 4)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeKnockout));

    new JoystickButton(rightController, 5)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kArmStow));

    new JoystickButton(rightController, 7)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kClimbAlign));
    new JoystickButton(rightController, 7).whileTrue(m_algaeSubsystem.climbAlignCommand());

    new JoystickButton(rightController, 6)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kClimb));
    new JoystickButton(rightController, 6).whileTrue(m_algaeSubsystem.climbCommand());

    new JoystickButton(rightController, 11) // Resets Odometry and sets gyro heading to 0
        .whileTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    new JoystickButton(leftController, 1).whileTrue(m_algaeSubsystem.reverseIntakeCommand());
    new JoystickButton(rightController, 1).whileTrue(m_algaeSubsystem.runIntakeCommand());
  }

  private Rotation2d getTargetAngleFromJoystick() {
    double xAngle = angleX.getAsDouble();
    double yAngle = angleY.getAsDouble();
    if (Math.abs(xAngle) >= OperatorConstants.DEADBAND
        || Math.abs(yAngle) >= OperatorConstants.DEADBAND) {
      Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(yAngle, xAngle));
      Constants.lastValidTargetAngle = targetAngle.minus(Rotation2d.fromDegrees(90));
    }
    return Constants.lastValidTargetAngle;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
