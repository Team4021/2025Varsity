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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  // Controller
  public static final Joystick leftController = new Joystick(1);
  public static final Joystick rightController = new Joystick(0);
  public static final Joystick buttonBox = new Joystick(2);

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
        break;
    }

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

    // Configure the button bindings
    configureButtonBindings();
    NamedCommands.registerCommand("Coral Outake", m_coralSubSystem.reverseIntakeCommand());
    NamedCommands.registerCommand(
        "Position three", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -leftController.getRawAxis(1),
            () -> leftController.getRawAxis(0),
            () -> rightController.getRawAxis(0)));

    // Note to self: removed negatives on x axis

    new JoystickButton(buttonBox, 1)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(0)));
    new JoystickButton(buttonBox, 2)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(300)));
    new JoystickButton(buttonBox, 3)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(240)));
    new JoystickButton(buttonBox, 4)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(180)));
    new JoystickButton(buttonBox, 5)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(-(2 * Math.PI) / 3)));
    new JoystickButton(buttonBox, 6)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(120)));
    new JoystickButton(buttonBox, 7)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(135)));
    new JoystickButton(buttonBox, 8)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(225)));
    new JoystickButton(buttonBox, 9)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -leftController.getRawAxis(1),
                () -> leftController.getRawAxis(0),
                () -> Rotation2d.fromDegrees(270)));

    // Switch to X pattern when X button is pressed
    // controlle.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Left Stick Button -> Set swerve to X

    // Left Bumper -> Run tube intake

    new JoystickButton(rightController, 1).whileTrue(m_coralSubSystem.runIntakeCommand());
    new JoystickButton(rightController, 2)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    new JoystickButton(rightController, 3)
        .whileTrue(
            m_coralSubSystem
                .setSetpointCommand(Setpoint.kFeederStation)
                .alongWith(m_algaeSubsystem.stowCommand()));
    new JoystickButton(rightController, 5)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    new JoystickButton(rightController, 4)
        .whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

    new JoystickButton(leftController, 1).whileTrue(m_coralSubSystem.reverseIntakeCommand());
    new JoystickButton(leftController, 4).whileTrue(m_algaeSubsystem.reverseIntakeCommand());
    new JoystickButton(leftController, 5).whileTrue(m_algaeSubsystem.runIntakeCommand());

    // if (leftController.getRawButton(4)) { // 4 = ?
    //     // Move the motor forward when the button is pressed
    //     m_algaeSubsystem.set( -0.3); // Adjust the speed (0.0 to 1.0) as needed
    // } else {
    //     // Stop the motor when the button is released
    //     m_algaeSubsystem.set( 0.0);
    // }
    // if (leftController.getRawButton(6)) { // 6 = ?
    //     // Move the motor forward when the button is pressed
    //     m_algaeSubsystem.set( 0.3); // Adjust the speed (0.0 to 1.0) as needed
    // } else {
    //     // Stop the motor when the button is released
    //     m_algaeSubsystem.set( 0.0);

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
