package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.VisionConstants;

/** Command that uses AprilTags to Align and Drive towards Coral Station */
public class leftCoralReefTrack extends Command {
  private LimelightSubsystem m_Limelight;
  public double aprilTagAlignment = 0;

  public PIDController alignedPID =
      new PIDController(
          VisionConstants.AlignmentConstants.kP,
          VisionConstants.AlignmentConstants.kI,
          VisionConstants.AlignmentConstants.kD);

  // The subsystem the command runs on
  // private final LimelightSubsystem m_LimelightSubsystem;
  public leftCoralReefTrack(LimelightSubsystem m_Limelight) {
    // m_LimelightSubsystem = subsystem;
    this.m_Limelight = m_Limelight;
    addRequirements(m_Limelight);
  }

  @Override
  public void initialize() {
    aprilTagAlignment = 0;
  }

  public void execute() {
    aprilTagAlignment =
        alignedPID.calculate(
            RobotContainer.Vision.getTargetX(0), RobotContainer.Vision.calculatecoralOffsetX());
  }

  @Override
  public boolean isFinished() {
    aprilTagAlignment = 0;
    return true;
  }
}
